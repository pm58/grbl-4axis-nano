# grbl-4axis-nano
grbl0.9
nano/uno
4axis xyza
根据grbl0.9版本改写的328-NANO-UNO的固件，支持XYZA四轴控制，因为328空间太小只有4k精简了很多而且写入后几乎是满的。做一点优化。后续升级A轴转换B轴或C轴，
建议使用grbl的控制通信软件：https://github.com/Denvi/Candle
这个软件控制速度高不卡，比其他的grbl软件运行和通信更流畅。特别是发送4轴运动代码时候更明显～！

提供一个固件，使用usbasp工具上传即可，支持NANO/UNO。
