# 📡 Moonraker 🌙

## Requirements

- Ruby 3.1.2
- Bundler 2.3.7
- ImageMagick

## Installation

```bash
# Change RubyGems install path to home dir
echo 'GEM_HOME="$HOME/.gems"' >> ~/.bashrc
source .bashrc

# Install Ruby
sudo apt install ruby-full

# Install deps
sudo apt-get install libmagickwand-dev

# Install gems
bundle
```

## Configuration

`cp etc/config.example.toml etc/config.toml` and edit the config file

## Calibration

TODO!

## Start Tracking

`bin/track`

Ctrl-C to exit.
