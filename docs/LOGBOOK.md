# Sept 18, 2025
**Members present:**  
all  
**Meeting duration:**  
2 hours  
**Meeting outcomes:**  
* consider using Raspberry Pi as microcontroller; Arduino may have insufficient processing power
* Time Difference of Arrival (TDoA) algorithm seems to be ideal for transient sound sources
* for constant sounds sources, need some way to find phase difference between microphones
* relative signal strength (RSS) method could work; based on relative intensity of sound at each microphone

# Sept 23, 2025
**Members present:**  
all  
**Meeting duration:**  
2 hours  
**Meeting outcomes:**  
* Could have the system use TDoA for transient sound source and RSS for constant sound source
* Can use an array of 3 microphones arranged in an equilateral traingle -- mics need to be less than 3 m away from microcontroller due to signal attenuation/noise over run of wire
  
# Oct 2, 2025
**Members present:**  
all  
**Meeting duration:**  
2 hours  
**Meeting outcomes:**  
* Got arduino uno and hardware from engineering lab
* Worked on preliminary project report; created content for most sections; still need to establish functions/requirements/means
* Consider using Raspberry Pi Pico; possible issue is that there is a single ADC, so ADC multiplexing would introduce a time skew into mic input timing data
* Consider using MEMS or similar digital microphones to bypass ADC multiplexing issues

# Oct 6, 2025
**Members present:**  
all  
**Meeting duration:**  
1 hour 
**Meeting outcomes:**  
* continued work on preliminary project report; created morphological chart

# Oct 9, 2025
**Members present:**  
all  
**Meeting duration:**  
2 hours  
**Meeting outcomes:**  
* finished preliminary project report
* Consider options for solving TDoA localization based on time delays between mics; consider Gauss-Newton iterative solver over geometric (hyperbolic solution) due to greater robustness
* purchased Pico; purchased 6x MAX4466 analog condenser mics

# Oct 14, 2025
**Members present:**  
all  
**Meeting duration:**  
2 hours  
**Meeting outcomes:**  
* Realized that RSS and TDoA are fundamentally incompatible; RSS requires a large mic array and TDoA requires a small array
* decided to pursue only TDoA for both transient and constant sound sources

# Oct 16, 2025
**Members present:**  
all  
**Meeting duration:**  
1 hours  
**Meeting outcomes:**  
* decided to move forward with initial prototype of equilateral triangular mic array with side length 20 cm
* started CAD modeling of mic and Pico mount

# Oct 21, 2025
**Members present:**  
all  
**Meeting duration:**  
2 hours  
**Meeting outcomes:**  
* printed prototype mount for mics & Pico
* did additional research into Gauss-Newton solver & built prototype code in C; found that the system tends to fall into local minima & catastrophic divergence when tested with synthetic data
* additional research showed that the Levenberg-Marquardt solver is a variation on Gauss-Newton solver incorporating damping factor $\lambda$ to improve robustness over wider range

# Oct 23, 2025
**Members present:**  
all  
**Meeting duration:**  
2 hours  
**Meeting outcomes:**  
* built prototype Levenberg-Marquardt solver; found that system returned good results with synthetic data when multiple initial guesses are taken at different distances along the axis of the geometrically calculated angle to the sound source
* assembled first hardware prototype
* decided to proceed first with detecting mic time deltas of transient sound sources; should be simpler than identifying mic time deltas from constant sound source, which will require more sophisticated cross-correlation algorithm --> extensive signal processing

# Oct 28, 2025
**Members present:**  
all  
**Meeting duration:**  
30 min
**Meeting outcomes:**  
* discussed plans for prototype testing in preparation from writing second report
* built prototype TDoA onset detector which accepts buffers of data from each of the 3 mics

# Nov 4, 2025
**Members present:**  
all  
**Meeting duration:**  
1 hour  
**Meeting outcomes:**  
* established that we would begin with the hardware implementation by implementing an algorithm which return which mic receives the threshold sound first
