# Kalico Telemetry

Kalico includes an optional telemetry system that collects anonymous information about your printer setup. This document explains what telemetry is, what data is collected, how it's used, and how to control it.

## What is Telemetry?

Telemetry is the collection of anonymous usage data that helps the Kalico project understand how users are using the software. This information is valuable for:

- Understanding which features are most commonly used
- Identifying which printer configurations are popular
- Prioritizing development efforts
- Improving compatibility with different hardware setups
- Making informed decisions about future development

## Opt-In Only

Kalico's telemetry system is **strictly opt-in**. When you first run Kalico, you'll be prompted to enable or disable telemetry. No data is collected until you explicitly enable it.

## What Data is Collected

Kalico telemetry collects the following anonymous information:

### Machine Identification
- A unique machine ID (either from `/etc/machine-id` or a generated UUID stored in `.machine-id`)
- This ID is used to avoid counting the same machine multiple times, but cannot be traced back to you

### System Information
- Hardware architecture (e.g., `x86_64`, `arm64`)
- Operating system information from `/etc/os-release` (distribution name, version)
- Python version
- CPU information (model name only)

### Kalico Software Information
- Kalico version number
- Git branch name
- Git remote URL

### Configuration Information
- **Only section and option names** from your configuration
- No actual configuration values are collected
- Example: The system knows you have a `[printer]` section with options like `max_velocity`, but not what values you've set

### Enabled Objects
- A list of all enabled Kalico modules and plugins
- This helps understand which features are being used

## What is NOT Collected

Kalico telemetry does **not** collect:

- Personal information (name, email, IP address)
- Actual configuration values or parameters
- Printer logs or performance data
- G-code files or print history
- Any information that could identify you personally

## How to Control Telemetry

### Viewing Your Telemetry Data

You can see exactly what data would be sent by using the `TELEMETRY_EXAMPLE` G-code command. This will save a file called `telemetry.json` in your configuration folder, containing the exact data that would be submitted.

### Enabling Telemetry

If you want to help improve Kalico by sharing anonymous usage data:

1. Run the `ENABLE_TELEMETRY` G-code command
2. Run `SAVE_CONFIG` to save the setting and restart the printer

### Disabling Telemetry

If you prefer not to share telemetry data:

1. Run the `DISABLE_TELEMETRY` G-code command
2. Run `SAVE_CONFIG` to save the setting and restart the printer

## Data Handling

- All telemetry data is sent to `https://telemetry.kalico.gg/collect`
- Data is transmitted securely via HTTPS
- Data is stored anonymously and aggregated for statistical purposes
- Individual machine data is not shared with third parties

## Configuration

Telemetry can be configured in your `printer.cfg` file:

```
[telemetry]
enabled: True   # Set to False to disable telemetry
```
