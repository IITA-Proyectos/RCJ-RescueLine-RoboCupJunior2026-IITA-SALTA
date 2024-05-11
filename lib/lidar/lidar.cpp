#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "lidar.h"

Lidar::Lidar(TwoWire *bus)
{
    this->_bus = bus;
    _bus->begin();
    _bus->setClock(400000); // use 400 kHz I2C
    _lidar.setBus(_bus);
    _lidar.setTimeout(500);
    _lidar.init();
    _lidar.setDistanceMode(VL53L1X::Medium);
    _lidar.setMeasurementTimingBudget(33000);
    _lidar.startContinuous(33);
}

int Lidar::getDist()
{
    if (_lidar.dataReady())
        _distance = _lidar.read(false);
    return _distance;
}