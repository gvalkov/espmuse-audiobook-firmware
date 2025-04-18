#include "player.h"

#include "FS.h"
#include "esp32-hal-log.h"

#include "utils.h"

using std::string;
using std::vector;

Book::Book(string dirname, Audio *audio) : dirname{dirname}, audio{audio}, idx_current{0}, initialized{false} {
    log_i("Loading book %s", dirname.c_str());
    readdir(dirname.c_str(), false, true, chapters);
    std::sort(chapters.begin(), chapters.end());

    // for (auto& chapter: chapters) {
    //     log_i("Chapter: %s", chapter.c_str());
    // }

    resume_path = dirname + ".txt";
    initialized = true;
}

Book::~Book() {
    audio->stopSong();
    log_i("Book %s destroyed", dirname.c_str());
}

bool Book::playChapter(uint idx, uint32_t pos) {
    if (idx >= chapters.size()) {
        log_e("Invalid chapter index: %d", idx);
        return false;
    }

    string path = chapters[idx];
    log_i("Playing chapter %d ('%s') at file position %d", idx, path.c_str(), pos);

    audio->connecttoFS(SD, path.c_str(), pos);

    idx_current = idx;
    return true;
}

bool Book::playChapter(uint idx) {
    return playChapter(idx, 0);
}

bool Book::playNext() {
    uint idx_next = idx_current + 1;
    return playChapter(idx_next);
}

bool Book::playPrev() {
    uint idx_prev = idx_current - 1;
    return playChapter(idx_prev);
}

bool Book::seek(int sec) {
    log_i("Seeking from %d/%d with %d", audio->getAudioCurrentTime(), audio->getTotalPlayingTime(), sec);
    return audio->setTimeOffset(sec);
}

bool Book::save() {
    uint32_t pos_sec = audio->getAudioCurrentTime()-2;
    uint32_t filepos = audio->getFilePos() - (audio->getBitRate() * 2 / 8) ;

    File fh = SD.open(resume_path.c_str(), FILE_WRITE);
    if (fh) {
        log_i("Saving position in '%s': %d %d %d", resume_path.c_str(), idx_current, pos_sec, filepos);
        fh.printf("%d %d %d\n", idx_current, pos_sec, filepos);
        fh.flush();
        fh.close();
        return true;
    } else {
        log_e("Error opening %s for writing", resume_path.c_str());
        return false;
    }
}

bool Book::resume() {
    File fh = SD.open(resume_path.c_str(), FILE_READ);

    if (!fh) {
        log_e("Error opening %s for reading", resume_path.c_str());
        return false;
    }

    char buffer[64];
    size_t len = fh.readBytesUntil('\n', buffer, sizeof(buffer)-1);
    buffer[len] = '\0';

    fh.close();

    uint idx = 0;
    uint pos = 0;
    uint pos_sec = 0;

    int parsed = sscanf(buffer, "%u %u %u", &idx, &pos_sec, &pos);
    if (parsed != 3) {
        log_e("Failed to parse '%s' from %s", buffer, resume_path.c_str());
        return false;
    }

    log_i("Resuming chapter at index %d at position %d (sec %d)", idx, pos, pos_sec);
    playChapter(idx, pos);

    return true;
}


Player::Player(Audio &audio) : audio{&audio}, current_book{nullptr}, initialized{false} {
    log_i("Listing root directory ...");
    readdir("/", true, false, book_titles);
    std::sort(book_titles.begin(), book_titles.end());

    for (auto& title: book_titles) {
        log_i("Title: %s", title.c_str());
    }

    initialized = true;
};

bool Player::loadBook(uint idx) {
    log_i("Loading book at index %d", idx);
    string dir = book_titles[idx];
    log_i("Title %s", dir.c_str());
    if (current_book)
        audio->stopSong();
    current_book = unique_ptr<Book> (new Book(dir, audio));
    current_book_idx = idx;
    return true;
}

bool Player::loadBook(uint idx, bool save_book) {
    bool res = loadBook(idx);
    if (save_book) 
        res &= save();
    return res;
}

bool Player::loadBook(uint idx, bool save_book, bool play_resume) {
    bool res = loadBook(idx);
    if (save_book) 
        res &= save();

    if (res && play_resume) {
        if (!current_book->resume()) {
            log_i("No saved position found. Loading first chapter ...");
            current_book->playChapter(0);
        }
    }

    return res;
}

bool Player::loadNext() {
    if (current_book_idx + 1 < book_titles.size()) {
        return loadBook(current_book_idx + 1, true, true);
    }
    log_i("No next book available.");
    return false;
}

bool Player::loadPrev() {
    if (current_book_idx == 0) {
        log_i("No previous book available.");
        return false;
    }
    return loadBook(current_book_idx - 1, true, true);
}

bool Player::save() {
    if (!current_book) {
        log_e("No book loaded.");
        return false;
    }

    const char* name = current_book->dirname.c_str();

    File fh = SD.open(resume_path, FILE_WRITE);
    if (!fh) {
        log_e("Error opening %s for writing", resume_path);
        return false;
    }

    log_i("Writing to %s: %s", resume_path, name);
    fh.print(name);
    fh.flush();
    fh.close();
    return true;
}

bool Player::resume() {
    File fh = SD.open(resume_path, FILE_READ);
    if (!fh) {
        log_e("Error opening %s for reading", resume_path);
        return false;
    }

    string saved_title;
    while (fh.available()) {
        char c = fh.read();
        saved_title += c;
    }
    fh.close();
    
    for (int i=0; i<book_titles.size(); ++i) {
        // log_i("%d, %s, %s", i, saved_title.c_str(), book_titles[i].c_str());
        // log_i("%d, %d, %d", i, saved_title.length(), book_titles[i].length());
        if (saved_title == book_titles[i]) {
            log_i("Resuming book %d: %s", i, saved_title.c_str());
            return loadBook(i, true);
        }
    }

    log_e("Unable to resume book: %s", saved_title.c_str());
    return false;
}
