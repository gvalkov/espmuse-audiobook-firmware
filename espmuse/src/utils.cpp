#include "utils.h"
#include "esp32-hal-log.h"

void listdir(const char *dirname, uint8_t levels) {
    log_i("Listing directory: %s", dirname);

    File root = SD.open(dirname);
    if (!root) {
        log_e("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        log_e("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            log_i("  DIR : %s", file.name());
            if (levels) {
                listdir(file.path(), levels - 1);
            }
        } else {
            log_i("  PATH: %s  SIZE: %s", file.path(), file.size());
        }
        file = root.openNextFile();
    }
}


bool readdir(const char* dirname, bool incdirs, bool incfiles, std::vector<std::string>& out) {
    File root = SD.open(dirname);

    if (!root) {
        log_e("Failed to open directory: %s", dirname);
        return false;
    }

    if (!root.isDirectory()) {
        log_e("Not a directory: %s", dirname);
        return false;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            if (incdirs) {
                out.insert(out.end(), string(file.path()));
            }
        } else {
            if (incfiles) {
                out.insert(out.end(), string(file.path()));
            }
        }
        file = root.openNextFile();
    }

    root.close();
    file.close();

    return true;
}