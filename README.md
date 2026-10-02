# racecar

A 1/10 scale RC car that learns to race by itself.

Put it on a track it has never seen. It drives one slow lap to map the track, then goes faster every lap after that. If I move the walls around it notices and starts over from the slow lap.

Only sensor is a 2D lidar. All the compute is on the car (Jetson Orin Nano).

Plan:

- write a small 2D simulator first and get everything working there
- then run the same code on the real car

Status: ordering parts. See [shopping-list.md](shopping-list.md).
