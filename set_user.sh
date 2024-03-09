#!/bin/bash

# Check if two arguments are provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <user.name> <user.email>"
    exit 1
fi

# Extract arguments
USERNAME="$1"
EMAIL="$2"

# Set git config for user.name and user.email
git config user.name "$USERNAME"
git config user.email "$EMAIL"

echo "Git user set to $USERNAME ($EMAIL)"