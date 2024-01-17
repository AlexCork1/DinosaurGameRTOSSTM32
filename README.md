# STM32 Dino Game Controller

This project turns your STM32F411 Discovery board into a game controller that allows you to play the Dino game on Chrome browser. The application utilizes the following concepts:<br />
- CMSIS v2 (abstraction layer above FreeRTOS)<br />
- The device represents itself to the computer as an HID device (keyboard)<br />
- Two ways to play the game:<br />
  - Clicking the user button on the board<br />
  - Connecting a potentiometer to pin PC5<br />
## Usage
1. **Button Control:**<br />
   - Press the user button on the Discovery board to make the Dino jump.<br />
2. **Potentiometer Control:**<br />
   - Connect a potentiometer to pin PC5.<br />
   - Rotate the potentiometer to control the Dino's jumps.<br />

## Implementation Details
The tasks are implemented in the `freertos.c` file, utilizing CMSISv2 (or more detailed FreeRTOS) for task management. The HID device emulation sends the space bar key to the computer whenever an event is triggered (rotation of the potentiometer or button press).

## Licence
This project is licensed under the MIT License.

## Contributing
Feel free to contribute by opening issues or pull requests. Your input is highly appreciated!

## Acknowledgments
Thanks to the STM32 community for their support and resources.
Happy gaming!
