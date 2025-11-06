## Calibration Data

### API
 
- Factory Provision Prepare
- Factory Provision Append Data
- Factory Provision Verify
- Factory Read Item


### Factory TLV's

Each section is dentoted by a 32-bit TAG, 32-bit LENGTH, ASCII Data, and CRC32 Checksum.

#### HWID

- Tag: `HWID`
- Format: JSON
- Contents:
    - ID Version
    - "Serial Number"
    - "Board Revision"
    - "Build Config"
    - "Build Date"
    - "Mfg Signature"


#### FCAL

- Tag: `FCAL`
- Format: JSON
- Contents:
    - Calibration Version
    - Calibration Station
    - Calibration Date
    - Calibration Data

