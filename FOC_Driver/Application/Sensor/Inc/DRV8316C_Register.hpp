// DRV8316C register map and bitfield positions.

#define REGISTER_LOCK_ADDR 0x03U

#define CTRL_REG4_ADDR 0x06U

#define CTRL_REG10_ADDR 0x0CU

// ---------------------------------------------------------------------------
// Control Register 10 (0x0C)
// Driver delay compensation enable:
// 0h = Disabled
// 1h = Enabled
#define DLYCMP_EN 4U
#define DLYCMP_EN_LENGTH 1U

// Delay target settings:
// 0h = 0 us
// 1h = 0.4 us
// 2h = 0.6 us
// 3h = 0.8 us
// 4h = 1.0 us
// 5h = 1.2 us
// 6h = 1.4 us
// 7h = 1.6 us
// 8h = 1.8 us
// 9h = 2.0 us
// Ah = 2.2 us
// Bh = 2.4 us
// Ch = 2.6 us
// Dh = 2.8 us
// Eh = 3.0 us
// Fh = 3.2 us
#define DLY_TARGET 0U
#define DLY_TARGET_LENGTH 4U
#define DLY_TARGET_0US 0x0U
#define DLY_TARGET_0P4US 0x1U
#define DLY_TARGET_0P6US 0x2U
#define DLY_TARGET_0P8US 0x3U
#define DLY_TARGET_1P0US 0x4U
#define DLY_TARGET_1P2US 0x5U
#define DLY_TARGET_1P4US 0x6U
#define DLY_TARGET_1P6US 0x7U
#define DLY_TARGET_1P8US 0x8U
#define DLY_TARGET_2P0US 0x9U
#define DLY_TARGET_2P2US 0xAU
#define DLY_TARGET_2P4US 0xBU
#define DLY_TARGET_2P6US 0xCU
#define DLY_TARGET_2P8US 0xDU
#define DLY_TARGET_3P0US 0xEU
#define DLY_TARGET_3P2US 0xFU

// ---------------------------------------------------------------------------
// Control Register 2 (0x04)
#define CTRL_REG2_ADDR 0x04U

// SDO mode setting:
// 0h = SDO IO in open-drain mode
// 1h = SDO IO in push-pull mode
#define SDO_MODE 5U
#define SDO_MODE_LENGTH 1U

// Slew rate settings:
// 0h = 25 V/us
// 1h = 50 V/us
// 2h = 125 V/us
// 3h = 200 V/us
#define SLEW 3U
#define SLEW_LENGTH 2U
#define SLEW_25V 0U
#define SLEW_50V 1U
#define SLEW_125V 2U
#define SLEW_200V 3U

// Device mode selection:
// 0h = 6x mode
// 1h = 6x mode with current limit
// 2h = 3x mode
// 3h = 3x mode with current limit
#define PWM_MODE 1U
#define PWM_MODE_LENGTH 2U

// Clear fault:
// 0h = No clear fault command is issued
// 1h = Clear latched fault bits (auto resets after write)
#define CLR_FLT 0U
#define CLR_FLT_LENGTH 1U

// ---------------------------------------------------------------------------
// Control Register 3 (0x05)
#define CTRL_REG3_ADDR 0x05U

// PWM frequency at 100% duty cycle:
// 0h = 20 kHz
// 1h = 40 kHz
#define PWM_100_DUTY_SEL 4U
#define PWM_100_DUTY_SEL_LENGTH 1U

// Overvoltage level selection:
// 0h = 34 V
// 1h = 22 V
#define OVP_SEL 3U
#define OVP_SEL_LENGTH 1U

// Overvoltage enable:
// 0h = Disabled
// 1h = Enabled
#define OVP_EN 2U
#define OVP_EN_LENGTH 1U

// SPI fault reporting disable:
// 0h = nFAULT reporting enabled
// 1h = nFAULT reporting disabled
#define SPI_FLT_REP 1U
#define SPI_FLT_REP_LENGTH 1U

// Overtemperature warning reporting:
// 0h = nFAULT reporting disabled
// 1h = nFAULT reporting enabled
#define OTW_REP 0U
#define OTW_REP_LENGTH 1U

// ---------------------------------------------------------------------------
// Control Register 5 (0x07)
#define CTRL_REG5_ADDR 0x07U

// Current limit recirculation:
// 0h = FETs (brake mode)
// 1h = Diodes (coast mode)
#define ILIM_RECIR 6U
#define ILIM_RECIR_LENGTH 1U

// Active asynchronous rectification enable:
// 0h = Disabled
// 1h = Enabled
#define EN_AAR 3U
#define EN_AAR_LENGTH 1U

// Active synchronous rectification enable:
// 0h = Disabled
// 1h = Enabled
#define EN_ASR 2U
#define EN_ASR_LENGTH 1U

// Current sense amplifier gain:
// 0h = 0.15 V/A
// 1h = 0.3 V/A
// 2h = 0.6 V/A
// 3h = 1.2 V/A
#define CSA_GAIN 0U
#define CSA_GAIN_LENGTH 2U

// ---------------------------------------------------------------------------
// Buck converter configuration (0x08)
#define BUCK_CONF_ADDR 0x08U

// Buck power sequencing disable:
// 0h = Enabled
// 1h = Disabled
#define BUCK_PS_DIS 4U
#define BUCK_PS_LENGTH 1U

// Buck current limit selection:
// 0h = 600 mA
// 1h = 150 mA
#define BUCK_CL 3U
#define BUCK_CL_LENGTH 1U

// Buck voltage selection:
// 0h = 3.3 V
// 1h = 5.0 V
// 2h = 4.0 V
// 3h = 5.7 V
#define BUCK_SEL 1U
#define BUCK_SEL_LENGTH 2U

// Buck disable:
// 0h = Enabled
// 1h = Disabled
#define BUCK_DIS 0U
#define BUCK_DIS_LENGTH 1U
