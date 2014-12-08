#include "FixedWingController.h"

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */


namespace FixedWingControllerDaemon
{
	/* oddly, ERROR is not defined for c++ */
	#ifdef ERROR
	# undef ERROR
	#endif
	static const int ERROR = -1;

FixedWingController::FixedWingController	*g_control = nullptr;
} // end namespace FixedWingControllerDaemon


extern "C" __EXPORT int FixedWingController_main(int argc, char *argv[]);

int FixedWingController_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: FixedWingControllerDaemon {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (FixedWingControllerDaemon::g_control != nullptr)
			errx(1, "already running");

		FixedWingControllerDaemon::g_control = new FixedWingController::FixedWingController;

		if (FixedWingControllerDaemon::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != FixedWingControllerDaemon::g_control->start()) {
			delete FixedWingControllerDaemon::g_control;
			FixedWingControllerDaemon::g_control = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (FixedWingControllerDaemon::g_control == nullptr || !FixedWingControllerDaemon::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (FixedWingControllerDaemon::g_control == nullptr)
			errx(1, "not running");

		delete FixedWingControllerDaemon::g_control;
		FixedWingControllerDaemon::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (FixedWingControllerDaemon::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}