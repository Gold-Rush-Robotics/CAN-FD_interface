# CAN-FD_interface
Official GRR Teensy Platformio microROS Zephyr repo


Motor Controller:
- void setMotorVelocity(float velocity);
- void setMotorEffort(int effort);
- float getMotorVelocity();
- int getMotorEffort();
- int getRPM();
- int getCurrent();
- bool isFaulted();
- std::string getJointName();
- std::string getControlType();

Can Interface:
- void init_can(void);
- std::string getDeviceInfo();
- void send_heartbeat(void);
- bool canfd_transport_open(struct uxrCustomTransport * transport);
- bool zephyr_transport_close(struct uxrCustomTransport * transport);
- size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t *buf, size_t len, uint8_t * err);
- size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);