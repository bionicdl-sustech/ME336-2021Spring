# Azure Kinect driver

## Installation

```bash
pip install open3d==0.8

# install fix for ubuntu 16, skip this if you are using ubuntu 18
pip install open3d_azure_kinect_ubuntu1604_fix

# Download https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules
sudo cp 99-k4a.rules /etc/udev/rules.d/
```

## Usage
```python
from deepclaw.driver.sensors.camera.AzureKinect import AzureKinect
camera  = AzureKinect()

# Sleep some time for the hardware to ready.
time.sleep(1)
frame = camera.get_frame()

cv2.imwrite("test_AzureKinect.png", frame.color_image[0])
print(camera.get_intrinsics())
```

## TODO
Currently, there is a bug in the driver. You have to use the default configuration. If you are using configuration read from the json file, you will counter the following error:
```
[Open3D INFO] AzureKinectSensor::Connect
[Open3D INFO] sensor_index 0
[Open3D INFO] Serial number: 000647492212
[Open3D INFO] Firmware build: Rel
[Open3D INFO] > Color: 1.6.102
[Open3D INFO] > Depth: 1.6.75[6109.7]
[2020-05-15 09:38:27.929] [error] [t=5125] ../../../src/usbcommand/usbcommand.c (838): usb_cmd_write(). Write command(80000001) ended in failure, Command status 0x00000001
[2020-05-15 09:38:27.929] [error] [t=5125] ../../../src/color_mcu/color_mcu.c (284): usb_cmd_write( colormcu->usb_cmd, DEV_CMD_SET_SYS_CFG, (uint8_t *)&sync_config, sizeof(sync_config), NULL, 0) returned failure in colormcu_set_multi_device_mode()
[2020-05-15 09:38:27.929] [error] [t=5125] ../../../src/sdk/k4a.c (879): colormcu_set_multi_device_mode(device->colormcu, config) returned failure in k4a_device_start_cameras()
[Open3D WARNING] Runtime error: k4a_plugin::k4a_device_set_color_control() failed
False
```