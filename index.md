## ECE 5725 Final Project: Osu! Autoplayer

You can use the [editor on GitHub](https://github.com/ryanmcmahon1/osu_autoplayer/edit/gh-pages/index.md) to maintain and preview the content for your website in Markdown files.

Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files.

### Mouse-Sharing Through Barrier and PyAutoGUI

One of the first challenges that we had to handle was sorting out how to control the mouse on the computer from the Raspberry Pi. After some investigation, we found that the Barrier application (https://github.com/debauchee/barrier) allows both mouse and keyboard to be shared between two devices without the need for a physical switch for sharing the input between the two devices. With the application installed on both devices, a wireless connection is established between the two devices, allowing the mouse to be dragged between screens and for the keyboard to be used on both screens. The only limitation of the Barrier application is that the application seems to only be executable through the Linux desktop, requiring the display to be open on the RPi end. Additionally, the version of the application on both devices had to match to work as expected, leading to the use of the 2.2 version being used as that is the one available on the RPi. In this version, the auto-config option did not work correctly for us, requiring that the IP address of the host device be indicated manually to set up the connection.
To control the mouse input through Python, we explored the PyAutoGUI library. This library supports capturing the mouse input from the desktop screen without requiring a specific window or clicks to detect the input. Through this library, the Python application is able to move the cursor to absolute and relative positions, as well as click down mouse buttons and control when the mouse button is pressed down. This library expects the DISPLAY variable in the OS environment to be set, which seems to only be the case when the Linux desktop view is open. When booting on the PiTFT on its own to the command line, this variable is not set and the library does not work as expected. Using this library further reinforces the need to have the Linux desktop display open on an external HDMI display to achieve the expected behavior.
We tested utilizing Barrier and PyAutoGUI together to control the mouse on the device connected to the RPi, and found that it is possible to control the mouse from the RPi on the secondary device with these libraries. For the Barrier setup, the RPi is set up as the server device, while the laptop to be controlled is set up as the client. PyAutoGUI has internal protections to prevent moving the mouse cursor outside the boundaries of the detected display; however, we need to exceed the boundaries of our display in order to set the mouse cursor position to the second device. The parameter FAILSAFE can be set to False to remove the exception raised from exceeding the coordinates of the main display, allowing the mouse cursor to move beyond the main display.  However, the detected mouse position from the PyAutoGUI library once the mouse cursor is on the secondary display shifts to the center of the screen, likely based on how Barrier manages the mouse sharing between the two devices. It is possible to instead control the mouse coordinates accurately through movement relative to the current mouse position, as long as no duration for the mouse movement is indicated. 
We utilized a program running on the RPi and a separate program running on the computer whose mouse is being controlled to test the setup. The program on the RPi utilizes the PyAutoGUI library to control the mouse to our desired location on the external screen, while the program on the computer prints out the mouse position on its own device to compare against the expected result. During development and testing of the project, we ran into some issues with consistent synchronization of the mouse position on screen with what was expected from the code. This was particularly the case on mouse movements across a large span of the screen. Combining these observations with the previous observation of the mouse coordinate on the RPi being detected at the center of the screen while controlling the cursor off-screen, we concluded that only movements up to half the size of the resolution of the display connected to the RPi can be done in one call. We suspect that Barrier polls the mouse position offset from the center to update the mouse position on the connected device, without supporting overloaded coordinates extending beyond the screen’s resolution. After learning about this issue, the mouse control within the program works around this limitation to make incremental movements as necessary to reach the correct mouse coordinates. Otherwise, the mouse sharing process went relatively smoothly.

## Screenshot Transmission over TCP using Sockets

To have the gameplay information reach the RPi, we decided to utilize Python to transmit screenshots of the game from the device running Osu! using Python networking code. With a host client running on the computer end and a client side running on the RPi, the two devices communicate with each other to transmit screenshots of the game. Starting from the example code from the official Python socket documentation (https://docs.python.org/3/library/socket.html#:~:text=override%20this%20setting.-,Example,-%C2%B6), we established a connection between the two devices utilizing TCP, using the IP addresses of each of the connected devices to the network. 

```python
test
import socket
```

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [Basic writing and formatting syntax](https://docs.github.com/en/github/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/ryanmcmahon1/osu_autoplayer/settings/pages). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://docs.github.com/categories/github-pages-basics/) or [contact support](https://support.github.com/contact) and we’ll help you sort it out.
