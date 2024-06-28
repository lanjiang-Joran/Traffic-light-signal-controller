功能：

实现四种不同的交通状况（无无、无有、有无、有有）和相应的灯光变换逻辑（主绿支红、主黄支红、支绿主红、支黄主红）。交通信号灯控制器需要根据不同的交通状况，自动切换不同的灯光状态，以避免交通拥堵和交通事故。例如，在无无情况下，主干道和支干道均无车辆通行，控制器需要保持原状态；在有无情况下，只有主干道有车辆通行，此时支干道应该停止通行，主干道应该放行；在无有情况下，只有支干道有车辆通行，此时主干道应该停止通行，支干道应该放行；在有有情况下，需要同时兼顾两条道路的通行情况，采用交替放行的方式。设计并实现车辆检测模拟系统，通过拨片开关模拟不同的交通流状态输入。车辆检测是智能交通灯控制器的重要组成部分。通过拨片开关模拟不同的交通流状态输入，可以有效地测试交通信号灯控制器在不同情况下的控制效果。例如，在无有情况下，如果控制器没有检测到主干道的车辆，可能会导致支干道一直保持红灯状态，从而影响交通效率。因此，设计并实现车辆检测模拟系统，可以有效地测试控制器的稳定性和可靠性。设计倒计时显示电路，以数码管形式展示主干道和支干道交通灯的剩余放行时间。倒计时显示电路是智能交通灯控制器的另一个重要组成部分。通过数码管形式展示主干道和支干道交通灯的剩余放行时间，可以提醒司机及时采取相应的行动，避免交通事故的发生。例如，在主干道绿灯将要结束时，倒计时显示电路可以提示未通过的车辆加快速度通过，从而减少交通拥堵。