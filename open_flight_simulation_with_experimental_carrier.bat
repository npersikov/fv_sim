C:
cd "C:\Program Files\FlightGear 2020.3"
SET FG_ROOT=C:\Program Files\FlightGear 2020.3\data
bin\fgfs.exe --aircraft=ExperimentalCarrrier --fdm=network,localhost,5501,5502,5503 --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --enable-freeze --airport=KSFO --runway=10L --altitude=328 --heading=0 --offset-distance=0 --offset-azimuth=0

cd C:\Users\Nikita\Documents\USC\Homework\Spring 2022\Flight Vehicle Stability and Control\fv_sim\fv_sim
run_simulation.m
fv_sim.slx