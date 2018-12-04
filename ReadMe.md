# Active tracker

## 12-4 update

- kcftracker: 
	- 判断跟丢与重新捕获 value < 0.5 & value <last_success_value - 0.15 & value < last_fail_value + 0.15
	- 附近的四个角加入判断