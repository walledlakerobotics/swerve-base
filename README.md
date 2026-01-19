# Walled Lake Robotics Java Swerve Template

## Description

A template project for an FRC swerve drivetrain that uses REV NEO Motors and CANcoders

Note that this template is designed for a drivetrain composed of four swerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO as the turning motor, and a CTRE CANcoder as the absolute turning encoder.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in Phoenix Tuner X.

## Features

* Full swerve drivetrain
* Path planning with PathPlanner
* Vision with PhotonVision

## Prerequisites
* CTRE-Phoenix (v6) v26.1.0
* PathplannerLib v2026.1.2
* PhotonLib v2026.1.1-rc-3
* REVLib v2026.0.1
* SPARK MAX Firmware v26.1.0
* Studica v2026.0.0

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Configs.java` and `Constants.java` files.
