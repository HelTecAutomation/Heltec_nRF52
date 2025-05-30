Installation instructions for openSUSE
======================================

- Install the latest Arduino IDE from [arduino.cc](https://www.arduino.cc/en/Main/Software).
- Open Terminal and execute the following command (copy->paste and hit enter):

  ```bash
  sudo usermod -a -G dialout $USER && \
  if [ `python --version 2>&1 | grep '2.7' | wc -l` = "1" ]; then \
  sudo zypper install git python-pip python-pyserial; \
  else \
  sudo zypper install git python3-pip python3-pyserial; \
  fi && \
  mkdir -p ~/Arduino/hardware/heltec && \
  cd ~/Arduino/hardware/heltec && \
  git clone https://github.com/HelTecAutomation/Heltec_nRF52.git && \
  cd Heltec_nRF52 && \
  git submodule update --init --recursive && \
  cd tools && \
  python get.py && \
  ```
- Restart Arduino IDE
