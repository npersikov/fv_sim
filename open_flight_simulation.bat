C:

cd C:\Users\Nikita\Documents\USC\Homework\Spring 2022\Flight Vehicle Stability and Control\fv_sim\fv_sim
matlab -r "edit 'C:\Users\Nikita\Documents\USC\Homework\Spring 2022\Flight Vehicle Stability and Control\fv_sim/fv_sim/run_simulation.m'; open_system('C:\Users\Nikita\Documents\USC\Homework\Spring 2022\Flight Vehicle Stability and Control\fv_sim/fv_sim/fv_sim.slx'); clc;"

cd "C:\Program Files\FlightGear 2020.3"
SET FG_ROOT=C:\Program Files\FlightGear 2020.3\data
bin\fgfs.exe --aircraft=ExperimentalCarrier --fdm=network,localhost,5501,5502,5503 --fog-fastest --disable-clouds --start-date-lat=2004:00:01:09:00:00 --disable-sound --in-air --enable-freeze --terrain-engine=pagedLOD --lod-levels="1 3 5 7 9" --enable-random-objects --enable-terrasync 
