#include <boost/asio.hpp>
#include <stdio.h>
#include <termio.h>
#include <unistd.h>

// arrows key definition
#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_LEFT 68
#define KEY_RIGHT 67
#define NUMPAD_1 52
#define NUMPAD_3 54
#define NUMPAD_5 69
#define NUMPAD_7 49
#define NUMPAD_9 53

// get char from user without pressing enter
// from: http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

int main()
{
	// set serial connection
	static boost::asio::io_service ios;
	boost::asio::serial_port sp(ios, "/dev/ttyACM0");
	sp.set_option(boost::asio::serial_port::baud_rate(9600));

	// type instruction
	std::cout << std::endl;
	std::cout << "Control Your Pirate4WD!" << std::endl;
	std::cout << "-----------------------" << std::endl;
	std::cout << "Moving around:" << std::endl;
	std::cout << "   q    w    e" << std::endl;
	std::cout << "   a    s    d" << std::endl;
	std::cout << "   z    x    c" << std::endl;
	std::cout << std::endl;
	std::cout << "Press + to quit" << std::endl;
	std::cout << std::endl;

	// receive input from user and execute
	char tav='s';
	std::string message("");
	message.assign(&tav);
	bool isCorrect = false;
	while('+' != tav) {
		tav = getch();
		isCorrect = false;
		switch(tav) {
			case KEY_UP:
				tav = 'w';
				message.assign(&tav);
				isCorrect = true;
				break;
			case KEY_DOWN:
				tav = 'x';
				message.assign(&tav);
				isCorrect = true;
				break;
			case KEY_LEFT:
				tav = 'a';
				message.assign(&tav);
				isCorrect = true;
				break;
			case KEY_RIGHT:
				tav = 'd';
				message.assign(&tav);
				isCorrect = true;
				break;
			case NUMPAD_1:
				tav = 'z';
				message.assign(&tav);
				isCorrect = true;
				break;
			case NUMPAD_3:
				tav = 'c';
				message.assign(&tav);
				isCorrect = true;
				break;
			case NUMPAD_5:
				tav = 's';
				message.assign(&tav);
				isCorrect = true;
				break;
			case NUMPAD_7:
				tav = 'q';
				message.assign(&tav);
				isCorrect = true;
				break;
			case NUMPAD_9:
				tav = 'e';
				message.assign(&tav);
				isCorrect = true;
				break;
			case 'q':
			case 'w':
			case 'e':
			case 'a':
			case 's':
			case 'd':
			case 'z':
			case 'x':
			case 'c':
				message.assign(&tav);
				isCorrect = true;
				break;
		}
		if (isCorrect)
			sp.write_some(boost::asio::buffer(message));
	}

	// stop the robot when quitting
	message = "s";
	sp.write_some(boost::asio::buffer(message));
	sp.close();

	return 0;
}
