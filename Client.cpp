#include <stdio.h>
#include <string.h>
#include <iostream>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>  
#include <unistd.h>
#include <signal.h>

int serial_file;

void signalHandler(int signum) {
    close(serial_file);
}

void printUsage(const std::string program_name) {
    std::cout << "Usage: " << program_name << " -p [path to UART]" << std::endl;
}

int main(int argc, char * argv[]) {
    
    if(argc != 3 || argv[1] != std::string("-p"))  {
        printUsage(argv[0]);
        return -1;
    }

    std::cout << "\n"
              << "\n"
              << "\n"
              << "   ______________________________SIMON MEMORY GAME_\n"
              << "                                                    \n"
              << "   ___________________ECSE 444_____________________ \n"
              << "\n"
              << "   _INSTRUCTIONS:__________________________________\n"
              << "\n"
              << "  A sequence of notes will be played out of the DAC\n"
              << "  speaker. The 1st sequence played is just composed\n"
              << "  of 1 note. Score a point for correctly whistling \n"
              << "  back the sequence to the microphone. A good guess\n"
              << "  adds another note to the sequence. A wrong guess \n"
              << "  leads to .. GAME OVER .. Best of luck!\n"
              << "\n"
              /*
                    << "_________________________________________________\n"
                    << "/   /   /   /   /   /   /   /   /   /   /   /   /\n"
                    << "_______Press the Blue Button to Start Game_______\n"
                    << "  /   /   /   /   /   /   /   /   /   /   /   /  \n"
                    << "_________________________________________________\n"
                */
              << "           ________________________________       \n"
              << "          |                                |      \n"
              << "          |   Press Blue Button to Start   |      \n"
              << "          |________________________________|      \n"
              << "\n"
              << std::endl;


    // Close file when user exits client
    signal(SIGINT, signalHandler);
    signal(SIGHUP, signalHandler);
    signal(SIGQUIT, signalHandler);

    // Open file path for UART
    serial_file = open(argv[2], O_RDWR);

    // Create struct to store port information and read information from port
    struct termios tty;
    if(tcgetattr(serial_file, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -2;
    }

    // Now we set UART options
    // These must correspond to the ones that we set for the MCU
    // More documentation: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // Set stop bit
    tty.c_cflag &= ~CSIZE; // Clear data size
    tty.c_cflag |= CS8; // Use 8-bit words
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD;

    tty.c_cc[VTIME] = 10; //Timeout from read (1 second)
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // We save our new settings to the UART
    if (tcsetattr(serial_file, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }


    // Allocate memory for read buffer, set size according to your needs
    while(true)  {

        char buffer [256];
        memset(&buffer, 0, sizeof(buffer));

        // Read from UART
        int num_bytes = read(serial_file, &buffer, sizeof(buffer));

        if (num_bytes < 0) {
            printf("Error reading: %s\n", strerror(errno));
            return 1;
        }

        printf("%s", buffer);

        // If have :, take user input to put initials
        if (std::string(buffer).find(":") != std::string::npos)  {
            std::string user_input;
            std::cin >> user_input;

            while(user_input.size() != 2) {
                user_input.clear();
                std::cin >> user_input;
            }
            write(serial_file, user_input.data(), user_input.size());
        // Clear screen
        } else if(std::string(buffer).find("#") != std::string::npos) {
            for (int i = 0; i < 50; i++) {
                std::cout << "\n";
            }
            std::cout << std::endl;
        }
    }
    close(serial_file);
    return 0;
}