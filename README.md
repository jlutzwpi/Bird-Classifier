# Bird-Classifier
ESP-32 CAM implementation of a Bird classifier using Arduino IDE

For the file server to run, you also need to have the [ESP32 SD File Manager](https://github.com/jameszah/ESPxWebFlMgr/tree/master/esp32_sd_file_manager) in your Arduino libraries in order for the sketch to compile.

This project uses an ESP32 Cam and an Edge Impulse model to determine if a bird is at your bird feeder.  If a bird is there, it will save the snapshot to the SD card.  The ESP SD File Manager can be accessed from a web browser to download (or delete) the photos that are stored on the SD card.
