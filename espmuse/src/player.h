#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <cstring>
#include <vector>

#include "Audio.h"
#include "FS.h"

using std::string;
using std::vector;

struct Book {
    string dirname;
    string resume_path;
    Audio *audio;
    bool initialized;

    vector<string> chapters;
    uint idx_current;

    Book(string dirname, Audio *audio);
    ~Book();

    bool playChapter(uint idx, uint32_t pos);
    bool playChapter(uint idx);
    bool playNext();
    bool playPrev();
    bool seek(int sec);

    bool save();
    bool resume();
};

struct Player {
    Player(Audio &audio);
    bool loadBook(uint idx);
    bool loadBook(uint idx, bool save);
    bool loadBook(uint idx, bool save, bool play_resume);
    bool loadNext();
    bool loadPrev();

    bool save();
    bool resume();

    vector<string> book_titles;
    unique_ptr<Book> current_book;
    uint current_book_idx;
    bool initialized;
    Audio *audio;
    const char* resume_path = "/current-book.txt";
};
