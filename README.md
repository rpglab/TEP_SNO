# Transmission Expansion Planning with seasonal Network Optimization (TEP w. SNO)
This repo implements three different Transmission Expansion Planning (TEP) models in AMPL. The test case used here is a modified IEEE 24-bus system; but these codes can work on any other systems.

(i) Model 1, TEP, a normal TEP model that is used as a Benchmark here.

(ii) Model 2, TEP w. SNO ("TEP-SNO-T1"), this TEP model optimizes the network topology for each season in the planning horizon. In this model, the status of new lines is NOT optimized. In other words, a new line, once it is constructed, is always considered to be in the network for all subsequent seasons.

(iii) Model 3, TEP w. Enhanced SNO ("TEP-SNO-T2"), this TEP model optimizes the network topology for each season in the planning horizon. In this model, the status of new lines is also optimized. In other words, a new line can be disconnected from the network for the subsequent seasons after its construction.

The test case used here is a modified IEEE RTS-96 reliability test system (24-bus) that was initially developed by the IEEE reliability subcommittee and published in 1979 and later enhanced in 1996. Reference: "The IEEE Reliability Test System-1996. A report prepared by the Reliability Test System Task Force of the Application of Probability Methods Subcommittee" and link: https://ieeexplore.ieee.org/document/780914.
Though only tested on this single system here, these codes can work on any other systems.

For simulation results provided here were obtained with the following computer: Intel Xeon W-2195 2.3GHz, 4.3GHz Turbo, 18C, 24.75M Cache, HT, (140W) DDR4-2666; 128GB (8x16GB) DDR4 2666MHz RDIMM ECC; M.2 512GB PCIe NVMe Class 40 Solid State Drive.

The following paper provides more details about these three models: 

<a class="off" href="https://ieeexplore.ieee.org/abstract/document/9087743" target="_blank">Xingpeng Li and Qianxue Xia, “Power System Expansion Planning with Seasonal Network Optimization,” Innovative Smart Grid Technologies (ISGT), Washington D.C., USA, Feb. 2020. (DOI: 10.1109/ISGT45199.2020.9087743)</a>


## Citation:
If you use these codes for your work, please cite the following paper:

Xingpeng Li and Qianxue Xia, “Power System Expansion Planning with Seasonal Network Optimization,” Innovative Smart Grid Technologies (ISGT), Washington D.C., USA, Feb. 2020. (DOI: 10.1109/ISGT45199.2020.9087743)


## Contact:
Dr. Xingpeng Li

University of Houston

Email: xli83@central.uh.edu

Website: https://rpglab.github.io/


## License:
This work is licensed under the terms of the Creative Commons Attribution 4.0 (CC BY 4.0) license. 
https://creativecommons.org/licenses/by/4.0/


## Disclaimer:
The authors do not make any warranty for the accuracy, completeness, or usefulness of any information disclosed; and the authors assume no liability or responsibility for any errors or omissions for the information (data/code/results etc) disclosed.

