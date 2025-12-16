# Workshop 3 Part 2: Session Notes
## Exercises 4-5: Remote Connectivity & mTLS Security

> **Session Date**: December 16, 2025
> **Container Used**: Option 2 - `workshop3-jazzy` (Jazzy + rmw_zenoh_cpp)

---

## Exercise 4: Remote Connectivity & Cloud Router

### Setup Used
- RealSense D435i camera for real data
- Zenoh router (`ros2 run rmw_zenoh_cpp rmw_zenohd`)
- 3 terminals: Router, Camera, Monitor

### Performance Results (720p @ 30 FPS)
```
$ ros2 topic bw /camera/camera/color/image_raw
84.07 MB/s from 30 messages
Message size mean: 2.76 MB min: 2.76 MB max: 2.76 MB

$ ros2 topic hz /camera/camera/color/image_raw
average rate: 30.016
min: 0.032s max: 0.035s std dev: 0.00059s
```

### Zenoh Scout Results
```bash
$ zenoh scout
Hello {
  zid: 260cde7aa68e85e5700ae6412cd7adf1,
  whatami: Peer,
  locators: [
    tcp/172.20.10.11:33429,
    tcp/[2401:4900:57b5:a1b4:...]:33429  # IPv6
  ]
}
```

### Config Files Examined

#### 1. SHM Client Config (`/workshop3/configs/zenoh_shm_client.json5`)
```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/127.0.0.1:7447"]
  },
  scouting: {
    multicast: { enabled: false },
    gossip: { enabled: true }
  },
  transport: {
    shared_memory: { enabled: true }
  }
}
```

#### 2. Bridge Router Config (`/workshop3/configs/bridge_router.json5`)
```json5
{
  mode: "router",
  listen: {
    endpoints: ["tcp/0.0.0.0:7447"]
  },
  scouting: {
    multicast: { enabled: false }
  }
}
```

#### 3. Bridge Client Config (`/workshop3/configs/bridge_client.json5`)
```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/ROBOT_IP:7447"]  // Replace with actual IP
  },
  scouting: {
    multicast: { enabled: true }
  }
}
```

### Key Concepts: The Three Pillars

| Pillar | Description | Examples |
|--------|-------------|----------|
| **Mode** | Role in the network | `client`, `peer`, `router` |
| **Connect/Listen** | How to find/accept connections | `tcp/127.0.0.1:7447`, `tcp/0.0.0.0:7447` |
| **Transport** | How data is moved | SHM, TCP, TLS, QUIC |

### Eureka Moment #1: Router Relays ALL Traffic

**Key Insight**: Unlike DDS where discovery leads to direct peer-to-peer connections, Zenoh router relays ALL traffic continuously.

```
DDS:    Discovery → Direct peer-to-peer (fails through NAT)
Zenoh:  All traffic through router (works through ANY NAT)
```

**Why this matters**: Both client and server connect OUTBOUND to the router. Neither needs to accept incoming connections. This is why Zenoh works through NAT/firewalls!

### Eureka Moment #2: One-Line Change for Remote

```json5
// LOCAL
connect: { endpoints: ["tcp/127.0.0.1:7447"] }

// REMOTE (just change the IP!)
connect: { endpoints: ["tcp/cloud-server-ip:7447"] }
```

---

## Exercise 5: mTLS Security

### Why mTLS?
- **TLS**: Server authenticated only (like HTTPS)
- **mTLS**: BOTH client AND server authenticate each other
- Essential for robot security - only authorized devices can connect

### Certificate Generation Commands (X.509 v3 Required!)

> **Important**: Zenoh requires X.509 v3 certificates. Basic OpenSSL commands create v1 certs which will fail with `UnsupportedCertVersion` error.

```bash
# Create directory
mkdir -p /tmp/zenoh-certs
cd /tmp/zenoh-certs

# Create v3 extension config (required for Zenoh!)
cat > v3.ext << 'EOF'
authorityKeyIdentifier=keyid,issuer
basicConstraints=CA:FALSE
keyUsage = digitalSignature, keyEncipherment
EOF

# 1. Generate CA (Certificate Authority) - with v3 extensions
openssl genrsa -out ca.key 2048
openssl req -x509 -new -nodes -key ca.key -sha256 -days 365 \
  -out ca.pem -subj "/CN=Zenoh-Workshop-CA" \
  -addext "basicConstraints=critical,CA:TRUE"

# 2. Generate Server Certificate (for Router)
openssl genrsa -out server.key 2048
openssl req -new -key server.key -out server.csr -subj "/CN=zenoh-router"
openssl x509 -req -in server.csr -CA ca.pem -CAkey ca.key \
  -CAcreateserial -out server.pem -days 365 -sha256 \
  -extfile v3.ext

# 3. Generate Client Certificate (for ROS nodes)
openssl genrsa -out client.key 2048
openssl req -new -key client.key -out client.csr -subj "/CN=ros2-client"
openssl x509 -req -in client.csr -CA ca.pem -CAkey ca.key \
  -CAcreateserial -out client.pem -days 365 -sha256 \
  -extfile v3.ext

# 4. Verify certificate version is v3
openssl x509 -in server.pem -text -noout | grep "Version"
# Should output: Version: 3 (0x2)

# Verify files created
ls -la /tmp/zenoh-certs/
```

### Generated Files
```
ca.key      - CA private key (keep secret!)
ca.pem      - CA certificate (distribute to all)
server.key  - Router private key
server.pem  - Router certificate
client.key  - Client private key
client.pem  - Client certificate
```

### TLS Config Files (Zenoh 1.7.1 Format)

**IMPORTANT**: Field names changed in Zenoh 1.7.1!

| Old Name (wrong) | New Name (correct) |
|------------------|-------------------|
| `root_ca_certificate_file` | `root_ca_certificate` |
| `server_private_key_file` | `listen_private_key` |
| `server_certificate_file` | `listen_certificate` |
| `client_private_key_file` | `connect_private_key` |
| `client_certificate_file` | `connect_certificate` |
| `client_auth: true` | `enable_mtls: true` |

#### Router TLS Config (`router_tls.json5`)
```json5
{
  mode: "router",
  listen: {
    endpoints: ["tls/0.0.0.0:7448"]
  },
  transport: {
    link: {
      tls: {
        root_ca_certificate: "/tmp/zenoh-certs/ca.pem",
        listen_private_key: "/tmp/zenoh-certs/server.key",
        listen_certificate: "/tmp/zenoh-certs/server.pem",
        enable_mtls: true
      }
    }
  }
}
```

#### Client TLS Config (`client_tls.json5`)
```json5
{
  mode: "client",
  connect: {
    endpoints: ["tls/127.0.0.1:7448"]
  },
  transport: {
    link: {
      tls: {
        root_ca_certificate: "/tmp/zenoh-certs/ca.pem",
        connect_private_key: "/tmp/zenoh-certs/client.key",
        connect_certificate: "/tmp/zenoh-certs/client.pem"
      }
    }
  }
}
```

### Certificate Storage Best Practices

> **⚠️ Workshop Note**: We used `/tmp/` for quick demos. This is temporary storage!

| Environment | Recommended Location |
|-------------|---------------------|
| **Development** | `./certs/` in project directory |
| **Linux Production** | `/etc/zenoh/certs/` (chmod 600) |
| **Docker** | Mount as volume from secure host path |
| **Kubernetes** | Mount as Secrets |

**Never store production certificates in `/tmp`!**

### Testing mTLS (Commands)

> **Important**: Global flags (`--config`, `--mode`) come BEFORE the subcommand!

```bash
# Start TLS Router (Terminal 1)
zenohd -c /tmp/zenoh-certs/router_tls.json5
# Expected output: "Zenoh can be reached at: tls/172.20.10.11:7448"

# Subscribe with TLS using Zenoh CLI (Terminal 2)
zenoh --config /tmp/zenoh-certs/client_tls.json5 subscribe -k "test/**"
# Output will be Base64 encoded

# Publish with TLS using Zenoh CLI (Terminal 3)
zenoh --config /tmp/zenoh-certs/client_tls.json5 put -k "test/secure" -v "Hello via mTLS!"

# Decode Base64 message
echo "SGVsbG8gdmlhIG1UTFMh" | base64 -d
# Output: "Hello via mTLS!"

# Test REJECTION (no certificate - should FAIL!)
zenoh --connect "tls/127.0.0.1:7448" subscribe -k "test/**"
# Expected: Connection fails, router shows "BadCertificate" warning
```

### Test Results (Verified Working!)
```
Terminal 1 (Router):   ✅ Listening on tls/172.20.10.11:7448
                       ⚠️ "BadCertificate" = blocking unauthorized connections!

Terminal 2 (Subscribe): ✅ Received: SGVsbG8gdmlhIG1UTFMh (base64 encoded)

Terminal 3 (Publish):   ✅ Message sent successfully
```

### Zenoh CLI Syntax
```bash
# Pattern: zenoh [global-flags] <subcommand> [subcommand-flags]
zenoh --config config.json5 subscribe -k "key/**"
zenoh --config config.json5 put -k "key/name" -v "value"
zenoh --config config.json5 get -k "key/**"
zenoh scout
zenohd -c config.json5                 # Start router
```

### Issues Encountered

1. **Wrong config field names**: Zenoh 1.7.1 uses different field names than older docs
   - Fix: Use `root_ca_certificate` not `root_ca_certificate_file`

2. **z_sub/z_pub not installed**: Zenoh CLI tools not in container
   - Fix: Use ROS 2 tools with `ZENOH_SESSION_CONFIG_URI` env var
   - Alternative: Install via `pip3 install eclipse-zenoh`

---

## Blog Content Ideas

### Diagrams to Include

1. **mTLS Handshake Diagram**
```
Client                              Server
   │                                   │
   │──── "Hello, here's my cert" ─────►│
   │◄─── "Here's MY cert too" ─────────│
   │     (Both verify each other)      │
   │◄════ Encrypted tunnel ═══════════►│
```

2. **NAT Traversal with Cloud Router**
```
Site A (NAT)        Cloud Router        Site B (NAT)
   │                    │                    │
   └──── outbound ──────┼────── outbound ────┘
        (both connect OUT, router relays)
```

3. **The Three Pillars Table**
```
| Mode | Connect/Listen | Transport |
| client/peer/router | endpoints | SHM/TCP/TLS |
```

### Key Takeaways for Blog

1. **Exercise 4**:
   - Zenoh router relays ALL traffic (not just discovery)
   - One-line change from local to remote connectivity
   - Cloud router pattern solves NAT traversal

2. **Exercise 5**:
   - mTLS = mutual authentication (both sides verify)
   - Certificate generation with OpenSSL
   - Zenoh 1.7.1 config field names
   - Storage best practices

---

## Next Steps

- [ ] Install Zenoh CLI tools for cleaner demos
- [ ] Complete mTLS test with working router
- [ ] Part 3: Exercises 6-8 (Wireless, Congestion, NAT)
- [ ] Update blog with session findings
