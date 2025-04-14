# Git and GitHub Workflow Guide

Git is the version control system used for this project, and GitHub is the platform where the remote repository is hosted. Understanding basic Git commands is crucial for collaboration and tracking changes.

## Basic Git Commands

* **Initialize a local repository**: `git init` (Creates a new Git repository in your current directory).
* **Clone a repository**: `git clone <repository-url>` (Creates a local copy of a remote repository).
* **Check status**: `git status` (Shows the state of your working directory and staging area).
* **Stage changes**:
    * `git add <file>` (Stages a specific file).
    * `git add .` (Stages all changes in the current directory and subdirectories).
* **Commit changes**: `git commit -m "Your concise commit message"` (Saves staged changes to the local repository).
* **Push changes**: `git push origin <branch-name>` (Uploads local commits to the specified branch on the remote repository named 'origin').
* **Pull changes**: `git pull origin <branch-name>` (Fetches changes from the remote branch and merges them into your current local branch).

## Clone vs. Pull vs. Push

* **Clone**: Use once to get a local copy of a remote repository.
* **Pull**: Use regularly to update your local copy with changes made by others.
* **Push**: Use to share your committed local changes with others via the remote repository.

## Branching

* **Switch branch**: `git checkout <branch-name>` (Switches to an existing branch).
* **Create and switch branch**: `git checkout -b <new-branch-name>` (Creates a new branch from your current location and switches to it).

## Connecting to GitHub with SSH (Recommended)

Using SSH keys avoids needing to enter your username and password repeatedly.

1.  **Generate SSH Key** (if you don't have one):
    ```bash
    ssh-keygen -t ed25519 -C "your_email@example.com"
    # Or use RSA: ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
    ```
    Accept defaults or set a passphrase if desired.

2.  **Start SSH Agent**:
    ```bash
    eval "$(ssh-agent -s)"
    ```

3.  **Add Key to Agent**:
    ```bash
    ssh-add ~/.ssh/id_ed25519 # Or id_rsa if you used RSA
    ```

4.  **Copy Public Key**:
    ```bash
    cat ~/.ssh/id_ed25519.pub # Or id_rsa.pub
    ```
    Copy the entire output.

5.  **Add Key to GitHub**:
    Go to GitHub > Settings > SSH and GPG keys > New SSH key. Paste the copied key, give it a title, and save.

## Remotes

* **Add a remote**: `git remote add origin <repository-url>` (Links your local repository to a remote one, typically named 'origin'. `git clone` does this automatically).
* **List remotes**: `git remote -v` (Shows the URLs of configured remotes).

## Command Summary

* `git init`
* `git clone <url>`
* `git status`
* `git add <file | .>`
* `git commit -m "message"`
* `git push origin <branch>`
* `git pull origin <branch>`
* `git checkout <branch>`
* `git checkout -b <new-branch>`
* `git remote add origin <url>`
* `ssh-keygen`, `ssh-add`, `cat ...pub` (for SSH setup)