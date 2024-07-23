Installation instructions for Fedora
=====================================

- Install the latest Arduino IDE from [arduino.cc](https://www.arduino.cc/en/Main/Software). `$ sudo dnf -y install arduino` will most likely install an older release.
- Open Terminal and execute the following command (copy->paste and hit enter):

  ```bash
  sudo usermod -a -G dialout $USER && \
  sudo dnf install git python3-pip python3-pyserial && \
  mkdir -p ~/Arduino/hardware/heltec && \
  cd ~/Arduino/hardware/heltec && \
  git clone https://github.com/HelTecAutomation/Heltec_nRF52.git && \
  cd Heltec_nRF52 && \
  git submodule update --init --recursive && \
  cd tools && \
  python get.py
  ```
- Restart Arduino IDE
