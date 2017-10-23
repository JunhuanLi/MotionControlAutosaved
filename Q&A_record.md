Description:
	This is a record of questions and answers encountered in the project of Auto Lawn Mower.

#1. 给一个与当前车体垂直的方向，但是车转过的角度不是直角
A： 首先排除两个轮子的转速差异。其次改小方向误差，可能没有等imu初始化完毕就开始控制，导致初始化方向其实不是（0，1）；判断为90度时的阈值过大；