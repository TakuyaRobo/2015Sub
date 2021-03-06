Sub: Sub.o RasPiDS3.o RasPiMS.o
	g++ -Wall -o Sub Sub.o RasPiDS3.o RasPiMS.o -lwiringPi -std=c++11 -pthread
RasPiMS.o: RasPiMS/RasPiMS.cpp
	g++ -Wall -c RasPiMS/RasPiMS.cpp -lwiringPi -std=c++11 -pthread
RasPiDS3.o: RasPiDS3/RasPiDS3.cpp
	g++ -Wall -c RasPiDS3/RasPiDS3.cpp -lwiringPi -std=c++11 -pthread
Sub.o: Sub.cpp
	g++ -Wall -c Sub.cpp -lwiringPi -std=c++11 -pthread
clean:
	rm -f *.o Sub
