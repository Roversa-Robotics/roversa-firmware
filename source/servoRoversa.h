#ifndef SERVOROVERSA_H
#define SERVOROVERSA_H

//servo parameters
#define MAX_US 2300
#define MIN_US 700
#define STOP_US 1500
#define FREQUENCY 50

void drive (int, int);
void forward (int, int);
void reverse (int, int);
void left (int, int);
void right(int, int);
void stop();
void neutral();

#endif // SERVOROVERSA_H
