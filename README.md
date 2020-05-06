# nrf52-ble-phy-coded-update-example

Example how to toggle the BLE coded / uncoded PHY on nRF52 SDK

Button 1:
* Switch to Uncoded PHY 1Mbps
* Switch to Uncoded PHY 2Mbps

```
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {
        case PHY_BUTTON_1BMPS:
                if (button_action == APP_BUTTON_PUSH)
                {
                        if (m_application_state.phy != APP_PHY_1M)
                        {
                                request_phy(0, BLE_GAP_PHY_1MBPS);
                                NRF_LOG_INFO("Request to do the PHY update 1MBPS");
                                m_application_state.phy = APP_PHY_1M;
                        }
                }
                break;

        case PHY_BUTTON_2BMPS:
                if (button_action == APP_BUTTON_PUSH)
                {
                        if (m_application_state.phy != APP_PHY_2M)
                        {
                                request_phy(0, BLE_GAP_PHY_2MBPS);
                                NRF_LOG_INFO("Request to do the PHY update 2MBPS");
                                m_application_state.phy = APP_PHY_2M;
                        }
                }
                break;
        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}
```

## Requirement
* NRF52832 DK Board x 1
* Mobile / nRF Connect Desktop
* SDK 16.0 / S132v7.0.1
* IDE Segger Embedded Studio

## Details
You can find all the details information at [URL](https://jimmywongiot.com/2020/05/06/how-to-work-with-ble-codec-1mbps-2mbps-and-codec-phy-on-nrf52-series/).

## Location
The project may need modifications to work with later versions or other boards.
To compile it, clone the repository in the /nRF5_SDK_XX.x.0/examples/ directory.
The application is built to be used with the official nRF5 SDK that can be downloaded from developer.nordicsemi.com
