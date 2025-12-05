# Contributing to Physical AI Laboratory

First off, thank you for considering contributing! This project aims to make robotics simulation accessible to everyone.

## Ways to Contribute

### 1. Report Issues

Found a bug or something confusing in the guide?

1. Check if the issue already exists
2. Open a new issue with:
   - Your system specs (OS, GPU, driver version)
   - Steps to reproduce
   - Expected vs actual behavior
   - Error messages (if any)

### 2. Suggest Improvements

Have ideas to make the guide better?

- Open an issue with the `enhancement` label
- Describe what you'd like to see
- Explain why it would help beginners

### 3. Fix Issues

Want to fix something yourself?

1. Fork the repository
2. Create a branch: `git checkout -b fix/issue-description`
3. Make your changes
4. Test locally (see below)
5. Submit a pull request

### 4. Add Content

Want to add new tutorials or sections?

- Tutorials for specific robots
- Troubleshooting for different hardware
- Tips and tricks you've learned

## Development Setup

### Prerequisites

- [Quarto CLI](https://quarto.org/docs/get-started/)
- Git

### Local Development

```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/physical-ai-lab.git
cd physical-ai-lab

# Preview the blog locally
cd blog
quarto preview
```

### Testing Changes

Before submitting a PR:

```bash
# Build the site
cd blog
quarto render

# Check for errors in the output
```

## Style Guidelines

### Writing Style

- **Be beginner-friendly**: Assume no prior knowledge
- **Explain the "why"**: Don't just show commands, explain what they do
- **Use visual markers**: 🎯 TL;DR, 📚 Concept, 🛠️ Hands-On, ✅ Expected, ⚠️ Warning
- **Include expected output**: Show what success looks like
- **Add troubleshooting**: Common errors and solutions

### Code Style

- Add comments explaining non-obvious commands
- Use consistent formatting
- Test all commands before submitting

### Commit Messages

- Use clear, descriptive commit messages
- Start with a verb: "Add", "Fix", "Update", "Remove"
- Reference issues when applicable: "Fix #123"

## Pull Request Process

1. Update documentation if needed
2. Test your changes locally
3. Create a pull request with:
   - Clear title
   - Description of changes
   - Link to related issues
4. Wait for review

## Code of Conduct

- Be respectful and inclusive
- Help beginners learn
- Give constructive feedback
- Assume good intentions

## Questions?

Open an issue with the `question` label, and we'll help you out!

---

Thank you for making robotics simulation more accessible! 🤖
