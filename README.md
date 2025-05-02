# RF_localization

This repository contains 2 ROS (noetic) packages that were developed for data acquisition of a radio frequency localization payload developed using RTK-GPS and a) RTL-SDR, b) Hack-RF

1) gps_sdr => Logs power and position data from RTL-SDR and RTK-GPS
2) gps_hrf => Logs power and position data from HackRF and RTK-GPS.

The collected data was used in implementing a localization algorithm, where the results narrowd to a localization accuracy of less than a meter.

# Payload setup
<div align="center">
    <image src = "images/Drone_with_payload.jpeg", width="50%" />
</div>


An Nvidia Jetson Orin Nano was used as an onboard computer, along with the RTK GPS (Ublox-F9P) and the RTL-SDR connected through serial interface.    

# ROS Package description

RTK GPS works by sending the RTCM corrections acquired from the base station to the rover GPS module. This allows the GPS measurements converge to an accuracy of ~14 mm. For the experimentation the GPS module were configured for 5 hz frequency. 

The RTL-SDR collects 3 million IQ samples per second. For synchronization of data, 60k samples were averaged and associated with every position measurement, allowing both sensor to work at same frequency.

Following graph represents structure of the framework. 

<div align="center">
    <image src="images/RQT_graph.png", width="50%" />
</div>

The RTK node reads the RTCM corrections from the radio connected to the base station and passes it to the rover module to get position data. The position data is poblished to 'relative_pos' topic by 'gps_node'.

The 'sdr_node' node averages the I & Q samples received and publishes it to 'sdr_power' topic. 

Receiving node - collects timestamped data from both of the above topics, time-synchronizes it and logs it to csv file.

<div align="center">
    <image src="images/3D_pos_error.jpg", width="45%">
</div>

The above figures shows test data where the RF source was located to an accuracy of less than a meter. The trajecotry implemented was to have multiple passes through and near the RF source making the localization more robust. 

This setup allowed efficient data acquisition with time synchronization of <0.1 second.
 
