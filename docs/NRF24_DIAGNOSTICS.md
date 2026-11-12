# nRF24L01+ bring-up and diagnostics (STM32F030F4P6)

This project uses the nRF24L01+ over SPI1. The driver is configured for a fixed
32-byte payload, 5-byte address, channel 10, 1 Mbps, 0 dBm, 2-byte CRC and
Enhanced ShockBurst Auto-ACK on pipe 0.

## Wiring used by this project

| nRF24L01+ | STM32F030F4P6 | Function |
|---|---|---|
| VCC | regulated 3.3 V | Power |
| GND | GND | Common ground |
| CE | PA3 | Radio TX/RX control |
| CSN | PA4 | SPI chip select (active low) |
| SCK | PA5 | SPI1 clock |
| MISO | PA6 | SPI1 input |
| MOSI | PA7 | SPI1 output |
| IRQ | not connected | Optional; polling is used |

Do not power the radio from 5 V. Place a 100 nF ceramic capacitor and a
low-ESR 10 uF capacitor close to the radio module's VCC/GND pins. The supply
must remain stable during RF transmit bursts. PA/LNA variants may need a
separate supply with substantially more peak-current margin.

## Fast debugger test

1. Build and flash the project, then start a debug session.
2. Add these symbols to the Watch window:
   - `watch_spi_config_ok`
   - `watch_spi_hal_ready`
   - `watch_spi_last_transfer_ok`
   - `watch_nrf_detected`
   - `watch_nrf_config_ok`
   - `watch_nrf_initialized`
   - `watch_nrf_link_ack`
   - `watch_nrf_tx_result`
   - `watch_nrf_init_stage`, `watch_nrf_init_error`
   - `watch_nrf_reg_status`, `watch_nrf_reg_rf_ch`
   - `watch_nrf_reg_config`, `watch_nrf_reg_en_aa`
   - `watch_nrf_reg_fifo_status`, `watch_nrf_reg_setup_retr`
   - `watch_nrf_tx_attempts`, `watch_nrf_tx_acks`
   - `watch_nrf_tx_max_rt`, `watch_nrf_tx_timeouts`
   - `watch_spi_transfers`, `watch_spi_errors`
   - `watch_nrf_status_errors`, `watch_nrf_status_error_streak`
3. Wait for `watch_nrf_detected == 1` and `watch_nrf_config_ok == 1`.
4. Set `watch_nrf_test_request = 1`. The firmware sends one packet whose
   first bytes are ASCII `NRF24_LINK_TEST`.
5. When `watch_nrf_test_done == 1`:
   - `watch_nrf_test_result == 1`: a valid RF ACK was received.
   - `watch_nrf_tx_result == 2`: MAX_RT; no ACK was received after retries.
   - `watch_nrf_tx_result == 3`: SPI/HAL transfer error.
   - `watch_nrf_tx_result == 4`: TX status did not arrive before timeout.

## How to interpret the checks

- `watch_spi_config_ok = 1`: STM32 SPI is configured for 8-bit frames.
- `watch_spi_last_transfer_ok = 1`: the last HAL SPI transfer completed.
  This alone does not prove that the external wires are correct.
- The driver rejects a STATUS byte whose reserved bit 7 is high. A floating MISO
  line often returns 0xFF; it is not allowed to masquerade as TX success.
- `watch_nrf_detected = 1`: the radio responded to SPI and passed two
  RF_CH write/read-back tests (0x2A and 0x55). This verifies the MCU SPI
  peripheral plus the connected radio's SPI register path.
- `watch_nrf_config_ok = 1`: configuration registers passed read-back checks.
- `watch_nrf_link_ack = 1`: the peer radio returned an Enhanced ShockBurst ACK.
  It does not prove that the receiver's application has consumed the payload.
- `watch_nrf_tx_result = 2` / MAX_RT usually means no compatible receiver is
  listening, or receiver channel/address/data rate/CRC/pipe/Auto-ACK does not
  match, or RF power/supply/antenna conditions are inadequate.

## Receiver must match

The other radio must be configured as a receiver with all of these matching
settings:

- RF channel: 10
- Data rate: 1 Mbps
- Address width: 5 bytes
- Address on RX pipe 0: `EE DD CC BB AA` (same byte order)
- 2-byte CRC enabled
- Auto-ACK enabled on pipe 0
- Fixed payload width: 32 bytes
- RX mode selected and CE held high

The transmitter sets both `TX_ADDR` and `RX_ADDR_P0` to the same five-byte
address so it can receive the ACK on pipe 0.

## Result codes

| `watch_nrf_tx_result` | Meaning |
|---:|---|
| 0 | No transmission result yet |
| 1 | TX_DS: remote radio ACK received |
| 2 | MAX_RT: retry limit reached without ACK |
| 3 | SPI transfer failed |
| 4 | TX completion timeout |
| 5 | Driver not initialized |
| 6 | Invalid argument |

## Engineering notes

- The nRF24L01+ uses one-byte SPI commands and an 8-bit SPI command set.
  SPI1 is therefore configured for 8-bit frames, mode 0, MSB first and
  4 MHz (32 MHz / prescaler 8).
- CE is held high for 1 ms for a deterministic pulse greater than the
  10 us minimum; the driver then polls STATUS with a finite timeout.
- Automatic retransmission is set to 15 retries with a 750 us delay.
- The driver retries initialization once per second while no radio is detected.
- No firmware can guarantee RF reception without a powered, correctly
  configured receiver and a suitable power supply, antenna and wiring.
