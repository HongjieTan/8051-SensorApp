# STM32F103C8-FreeRTOS-Keil

[![STM10C8-keil](https://img.shields.io/badge/8051-温度信息采集-brightgreen)](https://github.com/HongjieTan/8051-SensorApp)


## 内容列表

- [背景](#背景)
- [安装](#安装)
    - [方法一](#方法一)
    - [方法二](#方法二)
- [使用说明](#使用说明)
- [维护者](#维护者)
- [如何贡献](#如何贡献)

## 背景

初学80c51时有意做一点小项目，于是就尝试用Z-stack栈编写了采集温度信息并用zigbee协议传输然后使用uart传输至PC的小项目。


## 安装

### 方法一

方法一使用 [git](https://git-scm.com/) 来clone项目，请确保你本地安装了它们。

```sh
$ cd [mydir]
$ git clone https://github.com/HongjieTan/8051-SensorApp
```
### 方法二

方法二直接download本项目的zip包，下载完后解压即可。

## 使用说明

clone或下载解压后进入拷贝文件到你的CC2530库文件夹下，打开CC2530DB文件夹下的SensorApp.eww，编译时分别选择协调器和终端进行编译，然后将CC2530DB\CoordinatorEB\Exe下的.hex文件烧录到zigbee协调器，CC2530DB\EndDeviceEB\Exe下的.hex文件烧录到zigbee终端。

## 维护者

[@HongjieTan](https://github.com/HongjieTan)。

## 如何贡献

因为本人也是一名初学者难免会有疏漏，因此非常欢迎大家的指正！[提一个 Issue](https://github.com/HongjieTan/8051-SensorApp/issues) 或者提交一个 [Pull Request](https://github.com/HongjieTan/8051-SensorApp/pulls)。
