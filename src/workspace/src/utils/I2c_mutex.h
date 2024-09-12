#include <fcntl.h>		// For open()
#include <sys/file.h>	// For flock()
#include <unistd.h>		// For close()
#include <stdio.h>

namespace I2cSync{

	void lock_i2c() {
		int lock_fd = open("/var/lock/i2c.lock", O_CREAT, 0666);
		if (flock(lock_fd, LOCK_EX) != 0) {  // Waits for the lock
			perror("Failed to acquire I2C bus lock");
		}
	}

	void unlock_i2c() {
		int lock_fd = open("/var/lock/i2c.lock", O_CREAT, 0666);
		if (flock(lock_fd, LOCK_UN) != 0) {
			perror("Failed to release I2C bus lock");
		}
	}
	
	//LOCK_NB makes the lock acquisition non-blocking, meaning it returns immediately if the lock is unavailable.
	void lock_i2c_nb() {
		int lock_fd = open("/var/lock/i2c.lock", O_CREAT, 0666);
		if (flock(lock_fd, LOCK_NB) != 0) {  // Waits for the lock
			perror("Failed to acquire I2C bus lock");
		}
	}
}
