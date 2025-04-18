/*
 * Audio.cpp
 *
 *  Created on: Oct 28.2018
 *
 *  Version 3.0.13zc
 *  Updated on: Dec 29.2024
 *      Author: Wolle (schreibfaul1)
 *
 */
#include "Audio.h"
#include "mp3_decoder.h"

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AudioBuffer::AudioBuffer(size_t maxBlockSize) {
    if(maxBlockSize) m_resBuffSizeRAM = maxBlockSize;
    if(maxBlockSize) m_maxBlockSize = maxBlockSize;
}

AudioBuffer::~AudioBuffer() {
    if(m_buffer) free(m_buffer);
    m_buffer = NULL;
}

void AudioBuffer::setBufsize(int ram, int psram) {
    if(ram > -1) // -1 == default / no change
        m_buffSizeRAM = ram;
    if(psram > -1) m_buffSizePSRAM = psram;
}

int32_t AudioBuffer::getBufsize() { return m_buffSize; }

size_t AudioBuffer::init() {
    if(m_buffer) free(m_buffer);
    m_buffer = NULL;
    if(psramInit() && m_buffSizePSRAM > 0) { // PSRAM found, AudioBuffer will be allocated in PSRAM
        m_f_psram = true;
        m_buffSize = m_buffSizePSRAM;
        m_buffer = (uint8_t*)ps_calloc(m_buffSize, sizeof(uint8_t));
        m_buffSize = m_buffSizePSRAM - m_resBuffSizePSRAM;
    }
    if(m_buffer == NULL) { // PSRAM not found, not configured or not enough available
        m_f_psram = false;
        m_buffer = (uint8_t*)heap_caps_calloc(m_buffSizeRAM, sizeof(uint8_t), MALLOC_CAP_DEFAULT | MALLOC_CAP_INTERNAL);
        m_buffSize = m_buffSizeRAM - m_resBuffSizeRAM;
    }
    if(!m_buffer) return 0;
    m_f_init = true;
    resetBuffer();
    return m_buffSize;
}

void AudioBuffer::changeMaxBlockSize(uint16_t mbs) {
    m_maxBlockSize = mbs;
    return;
}

uint16_t AudioBuffer::getMaxBlockSize() { return m_maxBlockSize; }

size_t AudioBuffer::freeSpace() {
    if(m_readPtr == m_writePtr) {
        if(m_f_isEmpty == true) m_freeSpace = m_buffSize;
        else m_freeSpace = 0;
    }
    if(m_readPtr < m_writePtr) {
        m_freeSpace = (m_endPtr - m_writePtr) + (m_readPtr - m_buffer);
    }
    if(m_readPtr > m_writePtr) {
        m_freeSpace = m_readPtr - m_writePtr;
    }
    return m_freeSpace;
}

size_t AudioBuffer::writeSpace() {
    if(m_readPtr == m_writePtr) {
        if(m_f_isEmpty == true) m_writeSpace = m_endPtr - m_writePtr;
        else m_writeSpace = 0;
    }
    if(m_readPtr < m_writePtr) {
        m_writeSpace = m_endPtr - m_writePtr;
    }
    if(m_readPtr > m_writePtr) {
        m_writeSpace = m_readPtr - m_writePtr;
    }
    return m_writeSpace;
}

size_t AudioBuffer::bufferFilled() {
    if(m_readPtr == m_writePtr) {
        if(m_f_isEmpty == true) m_dataLength = 0;
        else m_dataLength = m_buffSize;
    }
    if(m_readPtr < m_writePtr) {
        m_dataLength = m_writePtr - m_readPtr;
    }
    if(m_readPtr > m_writePtr) {
        m_dataLength = (m_endPtr - m_readPtr) + (m_writePtr - m_buffer);
    }
    return m_dataLength;
}

size_t AudioBuffer::getMaxAvailableBytes() {
    if(m_readPtr == m_writePtr) {
    //   if(m_f_start)m_dataLength = 0;
        if(m_f_isEmpty == true) m_dataLength = 0;
        else m_dataLength = (m_endPtr - m_readPtr);
    }
    if(m_readPtr < m_writePtr) {
        m_dataLength = m_writePtr - m_readPtr;
    }
    if(m_readPtr > m_writePtr) {
        m_dataLength = (m_endPtr - m_readPtr);
    }
    return m_dataLength;
}

void AudioBuffer::bytesWritten(size_t bw) {
    if(!bw) return;
    m_writePtr += bw;
    if(m_writePtr == m_endPtr) { m_writePtr = m_buffer; }
    if(m_writePtr > m_endPtr) log_e("m_writePtr %i, m_endPtr %i", m_writePtr, m_endPtr);
    m_f_isEmpty = false;
}

void AudioBuffer::bytesWasRead(size_t br) {
    if(!br) return;
    m_readPtr += br;
    if(m_readPtr >= m_endPtr) {
        size_t tmp = m_readPtr - m_endPtr;
        m_readPtr = m_buffer + tmp;
    }
    if(m_readPtr == m_writePtr) m_f_isEmpty = true;
}

uint8_t* AudioBuffer::getWritePtr() { return m_writePtr; }

uint8_t* AudioBuffer::getReadPtr() {
    int32_t len = m_endPtr - m_readPtr;
    if(len < m_maxBlockSize) {                            // be sure the last frame is completed
        memcpy(m_endPtr, m_buffer, m_maxBlockSize - (len)); // cpy from m_buffer to m_endPtr with len
    }
    return m_readPtr;
}

void AudioBuffer::resetBuffer() {
    m_writePtr = m_buffer;
    m_readPtr = m_buffer;
    m_endPtr = m_buffer + m_buffSize;
    m_f_isEmpty = true;
}

uint32_t AudioBuffer::getWritePos() { return m_writePtr - m_buffer; }

uint32_t AudioBuffer::getReadPos() { return m_readPtr - m_buffer; }
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// clang-format off
Audio::Audio(bool internalDAC /* = false */, uint8_t channelEnabled /* = I2S_SLOT_MODE_STEREO */, uint8_t i2sPort) {

    mutex_playAudioData = xSemaphoreCreateMutex();
    mutex_audioTask     = xSemaphoreCreateMutex();

    m_chbufSize = 512 + 64;
    m_ibuffSize = 512 + 64;

    m_chbuf    = (char*) malloc(m_chbufSize);
    m_ibuff    = (char*) malloc(m_ibuffSize);
    m_outBuff  = (int16_t*)malloc(m_outbuffSize * sizeof(int16_t));
    if(!m_chbuf || !m_outBuff || !m_ibuff) log_e("oom");

#ifdef AUDIO_LOG
    m_f_Log = true;
#endif
#define AUDIO_INFO(...) { snprintf(m_ibuff, m_ibuffSize, __VA_ARGS__); if(audio_info) audio_info(m_ibuff); }

    m_f_channelEnabled = channelEnabled;
    m_f_internalDAC = internalDAC;
    m_i2s_num = i2sPort;  // i2s port number

    // -------- I2S configuration -------------------------------------------------------------------------------------------
#if ESP_IDF_VERSION_MAJOR == 5
    m_i2s_chan_cfg.id            = (i2s_port_t)m_i2s_num;  // I2S_NUM_AUTO, I2S_NUM_0, I2S_NUM_1
    m_i2s_chan_cfg.role          = I2S_ROLE_MASTER;        // I2S controller master role, bclk and lrc signal will be set to output
    m_i2s_chan_cfg.dma_desc_num  = 32;                     // number of DMA buffer
    m_i2s_chan_cfg.dma_frame_num = 256;                    // I2S frame number in one DMA buffer.
    m_i2s_chan_cfg.auto_clear    = true;                   // i2s will always send zero automatically if no data to send
    i2s_new_channel(&m_i2s_chan_cfg, &m_i2s_tx_handle, NULL);

    m_i2s_std_cfg.slot_cfg                = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO); // Set to enable bit shift in Philips mode
    m_i2s_std_cfg.gpio_cfg.bclk           = I2S_GPIO_UNUSED;           // BCLK, Assignment in setPinout()
    m_i2s_std_cfg.gpio_cfg.din            = I2S_GPIO_UNUSED;           // not used
    m_i2s_std_cfg.gpio_cfg.dout           = I2S_GPIO_UNUSED;           // DOUT, Assignment in setPinout()
    m_i2s_std_cfg.gpio_cfg.mclk           = I2S_GPIO_UNUSED;           // MCLK, Assignment in setPinout()
    m_i2s_std_cfg.gpio_cfg.ws             = I2S_GPIO_UNUSED;           // LRC,  Assignment in setPinout()
    m_i2s_std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
    m_i2s_std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
    m_i2s_std_cfg.gpio_cfg.invert_flags.ws_inv   = false;
    m_i2s_std_cfg.clk_cfg.sample_rate_hz = 44100;
    m_i2s_std_cfg.clk_cfg.clk_src        = I2S_CLK_SRC_DEFAULT;        // Select PLL_F160M as the default source clock
    m_i2s_std_cfg.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_512;      // mclk = sample_rate * 256
    i2s_channel_init_std_mode(m_i2s_tx_handle, &m_i2s_std_cfg);
    I2Sstart(m_i2s_num);
    m_sampleRate = 44100;

    if (internalDAC)  {
        #ifdef CONFIG_IDF_TARGET_ESP32  // ESP32S3 has no DAC
        printf("internal DAC is not supported");
         // no support in V5 ???
        #endif
    }
#else
    m_i2s_config.sample_rate          = 44100;
    m_i2s_config.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    m_i2s_config.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    m_i2s_config.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1; // interrupt priority
    m_i2s_config.dma_buf_count        = 16;
    m_i2s_config.dma_buf_len          = 512;
    m_i2s_config.use_apll             = APLL_DISABLE;
    m_i2s_config.tx_desc_auto_clear   = true;
    m_i2s_config.fixed_mclk           = true;
    m_i2s_config.mclk_multiple        = I2S_MCLK_MULTIPLE_128;

    if (internalDAC)  {
        #ifdef CONFIG_IDF_TARGET_ESP32  // ESP32S3 has no DAC
        printf("internal DAC");
        m_i2s_config.mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN );
        m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_MSB); // vers >= 2.0.5
        i2s_driver_install((i2s_port_t)m_i2s_num, &m_i2s_config, 0, NULL);
        i2s_set_dac_mode((i2s_dac_mode_t)m_f_channelEnabled);
        if(m_f_channelEnabled != I2S_DAC_CHANNEL_BOTH_EN) {
            m_f_forceMono = true;
        }
        #endif
    }
    else {
        m_i2s_config.mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S); // Arduino vers. > 2.0.0
        i2s_driver_install((i2s_port_t)m_i2s_num, &m_i2s_config, 0, NULL);
        m_f_forceMono = false;
    }
    i2s_zero_dma_buffer((i2s_port_t) m_i2s_num);

#endif // ESP_IDF_VERSION_MAJOR == 5
    for(int i = 0; i < 3; i++) {
        m_filter[i].a0 = 1;
        m_filter[i].a1 = 0;
        m_filter[i].a2 = 0;
        m_filter[i].b1 = 0;
        m_filter[i].b2 = 0;
    }
    computeLimit();  // first init, vol = 21, vol_steps = 21
    startAudioTask();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Audio::~Audio() {
    // I2Sstop(m_i2s_num);
    // InBuff.~AudioBuffer(); #215 the AudioBuffer is automatically destroyed by the destructor
    setDefaults();

#if ESP_IDF_VERSION_MAJOR == 5
    i2s_channel_disable(m_i2s_tx_handle);
    i2s_del_channel(m_i2s_tx_handle);
#else
    i2s_driver_uninstall((i2s_port_t)m_i2s_num); // #215 free I2S buffer
#endif

    x_ps_free(m_chbuf);
    x_ps_free(m_outBuff);
    x_ps_free(m_ibuff);

    stopAudioTask();
    vSemaphoreDelete(mutex_playAudioData);
    vSemaphoreDelete(mutex_audioTask);
}
// clang-format on
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::setBufsize(int rambuf_sz, int psrambuf_sz) {
    if(InBuff.isInitialized()) {
        log_e("Audio::setBufsize must not be called after audio is initialized");
        return;
    }
    InBuff.setBufsize(rambuf_sz, psrambuf_sz);
};

void Audio::initInBuff() {
    if(!InBuff.isInitialized()) {
        size_t size = InBuff.init();
        if(size > 0) { AUDIO_INFO("PSRAM %sfound, inputBufferSize: %u bytes", InBuff.havePSRAM() ? "" : "not ", size - 1); }
    }
    changeMaxBlockSize(1600); // default size mp3 or aac
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
esp_err_t Audio::I2Sstart(uint8_t i2s_num) {
#if ESP_IDF_VERSION_MAJOR == 5
    return i2s_channel_enable(m_i2s_tx_handle);
#else
    // It is not necessary to call this function after i2s_driver_install() (it is started automatically),
    // however it is necessary to call it after i2s_stop()
    return i2s_start((i2s_port_t)i2s_num);
#endif
}

esp_err_t Audio::I2Sstop(uint8_t i2s_num) {
#if ESP_IDF_VERSION_MAJOR == 5
    return i2s_channel_disable(m_i2s_tx_handle);
#else
    return i2s_stop((i2s_port_t)i2s_num);
#endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::setDefaults() {
    stopSong();
    initInBuff(); // initialize InputBuffer if not already done
    InBuff.resetBuffer();
    MP3Decoder_FreeBuffers();
    memset(m_outBuff, 0, m_outbuffSize * sizeof(int16_t)); // Clear OutputBuffer
    m_hashQueue.clear();
    m_hashQueue.shrink_to_fit(); // uint32_t vector
    AUDIO_INFO("buffers freed, free Heap: %lu bytes", (long unsigned int)ESP.getFreeHeap());

    m_f_firstmetabyte = false;
    m_f_playing = false;
    m_f_metadata = false;
    m_f_firstCall = true;        // InitSequence for processWebstream and processLocalFile
    m_f_firstCurTimeCall = true; // InitSequence for computeAudioTime
    m_f_firstPlayCall = true;    // InitSequence for playAudioData
    m_f_loop = false;     // Set if audio file should loop
    m_f_ts = false;
    m_f_stream = false;
    m_f_decode_ready = false;
    m_f_eof = false;
    m_f_lockInBuffer = false;
    m_f_acceptRanges = false;

    m_codec = CODEC_NONE;
    m_dataMode = AUDIO_NONE;
    m_audioCurrentTime = 0; // Reset playtimer
    m_audioFileDuration = 0;
    m_audioDataStart = 0;
    m_audioDataSize = 0;
    m_avr_bitrate = 0;     // the same as m_bitrate if CBR, median if VBR
    m_bitRate = 0;         // Bitrate still unknown
    m_bytesNotDecoded = 0; // counts all not decodable bytes
    m_curSample = 0;
    m_LFcount = 0;        // For end of header detection
    m_controlCounter = 0; // Status within readID3data() and readWaveHeader()
    m_channels = 2;       // assume stereo #209
    m_streamTitleHash = 0;
    m_fileSize = 0;
    m_haveNewFilePos = 0;
    m_validSamples = 0;
    m_sumBytesDecoded = 0;
    m_vuLeft = m_vuRight = 0; // #835
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::setFileLoop(bool input) {
    m_f_loop = input;
    return input;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::connecttoFS(fs::FS& fs, const char* path, int32_t fileStartPos) {
    xSemaphoreTakeRecursive(mutex_playAudioData, 0.3 * configTICK_RATE_HZ);
    bool res = false;
    int16_t dotPos;
    char* audioPath = NULL;
    m_fileStartPos = fileStartPos;
    uint8_t codec = CODEC_NONE;

    if (!path) {
        printProcessLog(AUDIOLOG_PATH_IS_NULL);
        goto exit;
    }  // guard

    dotPos = lastIndexOf(path, ".");
    if (dotPos == -1) {
        AUDIO_INFO("No file extension found");
        goto exit;
    }  // guard
    setDefaults(); // free buffers an set defaults

    if (endsWith(path, ".mp3")) codec = CODEC_MP3;
    if (endsWith(path, ".wav")) codec = CODEC_WAV;
    if (codec == CODEC_NONE) {
        AUDIO_INFO("The %s format is not supported", path + dotPos);
        goto exit;
    }  // guard

    audioPath = (char *)x_ps_calloc(strlen(path) + 2, sizeof(char));
    if (!audioPath) {
        printProcessLog(AUDIOLOG_OUT_OF_MEMORY);
        goto exit;
    };
    if (path[0] != '/') {
        audioPath[0] = '/';
    }
    strcat(audioPath, path);

    if (!fs.exists(audioPath)) {
        printProcessLog(AUDIOLOG_FILE_NOT_FOUND, audioPath);
        goto exit;
    }
    AUDIO_INFO("Reading file: \"%s\"", audioPath);
    audiofile = fs.open(audioPath);
    m_dataMode = AUDIO_LOCALFILE;
    m_fileSize = audiofile.size();

    res = initializeDecoder(codec);
    m_codec = codec;
    if (res)
        m_f_running = true;
    else
        audiofile.close();

exit:
    free(audioPath);
    xSemaphoreGiveRecursive(mutex_playAudioData);
    return res;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
size_t Audio::readAudioHeader(uint32_t bytes) {
    size_t bytesReaded = 0;
    if(m_codec == CODEC_WAV) {
        int res = read_WAV_Header(InBuff.getReadPtr(), bytes);
        if(res >= 0) bytesReaded = res;
        else { // error, skip header
            m_controlCounter = 100;
        }
    }
    if(m_codec == CODEC_MP3) {
        m_controlCounter = 100;
    }
    if(!isRunning()) {
        log_e("Processing stopped due to invalid audio header");
        return 0;
    }
    return bytesReaded;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int Audio::read_WAV_Header(uint8_t* data, size_t len) {
    static size_t   headerSize;
    static uint32_t cs = 0;
    static uint8_t  bts = 0;

    if(m_controlCounter == 0) {
        m_controlCounter++;
        if((*data != 'R') || (*(data + 1) != 'I') || (*(data + 2) != 'F') || (*(data + 3) != 'F')) {
            AUDIO_INFO("file has no RIFF tag");
            headerSize = 0;
            return -1; // false;
        }
        else {
            headerSize = 4;
            return 4; // ok
        }
    }

    if(m_controlCounter == 1) {
        m_controlCounter++;
        cs = (uint32_t)(*data + (*(data + 1) << 8) + (*(data + 2) << 16) + (*(data + 3) << 24) - 8);
        headerSize += 4;
        return 4; // ok
    }

    if(m_controlCounter == 2) {
        m_controlCounter++;
        if((*data != 'W') || (*(data + 1) != 'A') || (*(data + 2) != 'V') || (*(data + 3) != 'E')) {
            AUDIO_INFO("format tag is not WAVE");
            return -1; // false;
        }
        else {
            headerSize += 4;
            return 4;
        }
    }

    if(m_controlCounter == 3) {
        if((*data == 'f') && (*(data + 1) == 'm') && (*(data + 2) == 't')) {
            m_controlCounter++;
            headerSize += 4;
            return 4;
        }
        else {
            headerSize += 4;
            return 4;
        }
    }

    if(m_controlCounter == 4) {
        m_controlCounter++;
        cs = (uint32_t)(*data + (*(data + 1) << 8));
        if(cs > 40) return -1; // false, something going wrong
        bts = cs - 16;         // bytes to skip if fmt chunk is >16
        headerSize += 4;
        return 4;
    }

    if(m_controlCounter == 5) {
        m_controlCounter++;
        uint16_t fc = (uint16_t)(*(data + 0) + (*(data + 1) << 8));                                               // Format code
        uint16_t nic = (uint16_t)(*(data + 2) + (*(data + 3) << 8));                                              // Number of interleaved channels
        uint32_t sr = (uint32_t)(*(data + 4) + (*(data + 5) << 8) + (*(data + 6) << 16) + (*(data + 7) << 24));   // Samplerate
        uint32_t dr = (uint32_t)(*(data + 8) + (*(data + 9) << 8) + (*(data + 10) << 16) + (*(data + 11) << 24)); // Datarate
        uint16_t dbs = (uint16_t)(*(data + 12) + (*(data + 13) << 8));                                            // Data block size
        uint16_t bps = (uint16_t)(*(data + 14) + (*(data + 15) << 8));                                            // Bits per sample

        AUDIO_INFO("FormatCode: %u", fc);
        // AUDIO_INFO("Channel: %u", nic);
        // AUDIO_INFO("SampleRate: %u", sr);
        AUDIO_INFO("DataRate: %lu", (long unsigned int)dr);
        AUDIO_INFO("DataBlockSize: %u", dbs);
        AUDIO_INFO("BitsPerSample: %u", bps);

        if((bps != 8) && (bps != 16)) {
            AUDIO_INFO("BitsPerSample is %u,  must be 8 or 16", bps);
            stopSong();
            return -1;
        }
        if((nic != 1) && (nic != 2)) {
            AUDIO_INFO("num channels is %u,  must be 1 or 2", nic);
            stopSong();
            return -1;
        }
        if(fc != 1) {
            AUDIO_INFO("format code is not 1 (PCM)");
            stopSong();
            return -1; // false;
        }
        setBitsPerSample(bps);
        setChannels(nic);
        setSampleRate(sr);
        setBitrate(nic * sr * bps);
        //    AUDIO_INFO("BitRate: %u", m_bitRate);
        headerSize += 16;
        return 16; // ok
    }

    if(m_controlCounter == 6) {
        m_controlCounter++;
        headerSize += bts;
        return bts; // skip to data
    }

    if(m_controlCounter == 7) {
        if((*(data + 0) == 'd') && (*(data + 1) == 'a') && (*(data + 2) == 't') && (*(data + 3) == 'a')) {
            m_controlCounter++;
            //    vTaskDelay(30);
            headerSize += 4;
            return 4;
        }
        else {
            headerSize++;
            return 1;
        }
    }

    if(m_controlCounter == 8) {
        m_controlCounter++;
        size_t cs = *(data + 0) + (*(data + 1) << 8) + (*(data + 2) << 16) + (*(data + 3) << 24); // read chunkSize
        headerSize += 4;
        if (cs) {
            m_audioDataSize = cs - 44;
        } else {  // sometimes there is nothing here
            if(m_dataMode == AUDIO_LOCALFILE) m_audioDataSize = getFileSize() - headerSize;
        }
        AUDIO_INFO("Audio-Length: %u", m_audioDataSize);
        return 4;
    }
    m_controlCounter = 100; // header succesfully read
    m_audioDataStart = headerSize;
    return 0;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::stopSong() {
    m_f_lockInBuffer = true; // wait for the decoding to finish
        static uint8_t maxWait = 0;
        while(m_f_audioTaskIsDecoding) {vTaskDelay(1); maxWait++; if(maxWait > 100) break;} // in case of error wait max 100ms
        maxWait = 0;
        uint32_t pos = 0;
        if(m_f_running) {
            m_f_running = false;
            if(m_dataMode == AUDIO_LOCALFILE) {
                pos = getFilePos() - inBufferFilled();
            }
        }
        if(audiofile) {
            // added this before putting 'm_f_localfile = false' in stopSong(); shoulf never occur....
            AUDIO_INFO("Closing audio file \"%s\"", audiofile.name());
            audiofile.close();
        }
        memset(m_filterBuff, 0, sizeof(m_filterBuff)); // Clear FilterBuffer
        m_validSamples = 0;
        m_audioCurrentTime = 0;
        m_audioFileDuration = 0;
        m_codec = CODEC_NONE;
        m_dataMode = AUDIO_NONE;
        m_f_lockInBuffer = false;
    return pos;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::pauseResume() {
    xSemaphoreTake(mutex_audioTask, 0.3 * configTICK_RATE_HZ);
    bool retVal = false;
    if(m_dataMode == AUDIO_LOCALFILE) {
        m_f_running = !m_f_running;
        retVal = true;
        if(!m_f_running) {
            memset(m_outBuff, 0, m_outbuffSize * sizeof(int16_t)); // Clear OutputBuffer
            m_validSamples = 0;
        }
    }
    xSemaphoreGive(mutex_audioTask);
    return retVal;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::playChunk() {
    int16_t validSamples = 0;
    static uint16_t count = 0;
    size_t i2s_bytesConsumed = 0;
    int16_t* sample[2] = {0};
    int16_t* s2;
    int sampleSize = 4; // 2 bytes per sample (int16_t) * 2 channels
    esp_err_t err = ESP_OK;
    int i= 0;

    if(count > 0) goto i2swrite;

    if(getChannels() == 1){
        for (int i = m_validSamples - 1; i >= 0; --i) {
            int16_t sample = m_outBuff[i];
            m_outBuff[2 * i] = sample;
            m_outBuff[2 * i + 1] = sample;
        }
    //    m_validSamples *= 2;
    }

    validSamples = m_validSamples;

    while(validSamples) {
        *sample = m_outBuff + i;
        computeVUlevel(*sample);

        if (m_f_forceMono && m_channels == 2) {
            int32_t xy = ((*sample)[RIGHTCHANNEL] + (*sample)[LEFTCHANNEL]) / 2;
            (*sample)[RIGHTCHANNEL] = (int16_t)xy;
            (*sample)[LEFTCHANNEL]  = (int16_t)xy;
        }
        Gain(*sample);

        if (m_f_internalDAC) {
            s2 = *sample;
            s2[LEFTCHANNEL] += 0x8000;
            s2[RIGHTCHANNEL] += 0x8000;
        }
        i += 2;
        validSamples -= 1;
    }
    if (audio_process_i2s) {
        // processing the audio samples from external before forwarding them to i2s
        bool continueI2S = false;
        audio_process_i2s((int16_t*)m_outBuff, m_validSamples, 16, 2, &continueI2S);
        if (!continueI2S) {
            m_validSamples = 0;
            count = 0;
            return;
        }
    }

i2swrite:

    validSamples = m_validSamples;


#if(ESP_IDF_VERSION_MAJOR == 5)
    err = i2s_channel_write(m_i2s_tx_handle, (int16_t*)m_outBuff + count, validSamples * sampleSize, &i2s_bytesConsumed, 10);
#else
    err = i2s_write((i2s_port_t)m_i2s_num, (int16_t*)m_outBuff + count, validSamples * sampleSize, &i2s_bytesConsumed, 10);
#endif

    if( ! (err == ESP_OK || err == ESP_ERR_TIMEOUT)) goto exit;
    m_validSamples -= i2s_bytesConsumed / sampleSize;
    count += i2s_bytesConsumed / 2;
    if(m_validSamples < 0) { m_validSamples = 0; }
    if(m_validSamples == 0) { count = 0; }

// ---- statistics, bytes written to I2S (every 10s)
    // static int cnt = 0;
    // static uint32_t t = millis();

    // if(t + 10000 < millis()){
    //     log_w("%i", cnt);
    //     cnt = 0;
    //     t = millis();
    // }
    // cnt+= i2s_bytesConsumed;
//-------------------------------------------


    return;
exit:
    if     (err == ESP_OK) return;
    else if(err == ESP_ERR_INVALID_ARG)   log_e("NULL pointer or this handle is not tx handle");
    else if(err == ESP_ERR_TIMEOUT)       log_e("Writing timeout, no writing event received from ISR within ticks_to_wait");
    else if(err == ESP_ERR_INVALID_STATE) log_e("I2S is not ready to write");
    else log_e("i2s err %i", err);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::loop() {
    if(!m_f_running) return;

    switch(m_dataMode) {
        case AUDIO_LOCALFILE:
            processLocalFile(); break;
    }
}

void Audio::processLocalFile() {
    if(!(audiofile && m_f_running && m_dataMode == AUDIO_LOCALFILE)) return; // guard

    static uint32_t ctime = 0;
    const uint32_t  timeout = 8000;                          // ms
    const uint32_t  maxFrameSize = InBuff.getMaxBlockSize(); // every mp3/aac frame is not bigger
    uint32_t        availableBytes = 0;

    if(m_f_firstCall) { // runs only one time per connection, prepare for start
        m_f_firstCall = false;
        m_f_stream = false;
        ctime = millis();
        uint32_t pos = audiofile.position();
        audiofile.seek(pos);
        m_audioDataSize = audiofile.size();
        if(m_resumeFilePos == 0) m_resumeFilePos = -1; // parkposition
        return;
    }
    availableBytes = InBuff.writeSpace();
    int32_t bytesAddedToBuffer = audiofile.read(InBuff.getWritePtr(), availableBytes);
    if(bytesAddedToBuffer > 0) {InBuff.bytesWritten(bytesAddedToBuffer);}
    if(!m_f_stream) {
        if(m_controlCounter != 100) {
            if((millis() - ctime) > timeout) {
                log_e("audioHeader reading timeout");
                m_f_running = false;
                return;
            }
            if(InBuff.bufferFilled() > maxFrameSize || (InBuff.bufferFilled() == m_fileSize)) { // at least one complete frame or the file is smaller
                InBuff.bytesWasRead(readAudioHeader(InBuff.getMaxAvailableBytes()));
            }
            return;
        }
        else {
            m_f_stream = true;
            AUDIO_INFO("stream ready");
        }
    }

    if(m_fileStartPos > 0){
        setFilePos(m_fileStartPos);
        m_fileStartPos = -1;
    }

    if(m_resumeFilePos >= 0) {
        if(m_resumeFilePos <  (int32_t)m_audioDataStart) m_resumeFilePos = m_audioDataStart;
        if(m_resumeFilePos >= (int32_t)m_audioDataStart + m_audioDataSize) {goto exit;}
        m_haveNewFilePos = m_resumeFilePos;

        if (m_codec == CODEC_WAV) {
            while ((m_resumeFilePos % 4) != 0) {
                m_resumeFilePos++;
                if (m_resumeFilePos >= m_fileSize) goto exit;
            }
        }  // must divisible by four

        if (m_codec == CODEC_MP3) {
            m_resumeFilePos = mp3_correctResumeFilePos(m_resumeFilePos);
            if (m_resumeFilePos == -1) goto exit;
            MP3Decoder_ClearBuffer();
        }

        m_f_lockInBuffer = true;                          // lock the buffer, the InBuffer must not be re-entered in playAudioData()
            while(m_f_audioTaskIsDecoding) vTaskDelay(1); // We can't reset the InBuffer while the decoding is in progress
            audiofile.seek(m_resumeFilePos);
            InBuff.resetBuffer();
            m_sumBytesDecoded = m_haveNewFilePos = m_resumeFilePos;
            m_resumeFilePos = -1;
            if(m_codec == CODEC_MP3) MP3Decoder_ClearBuffer();
        m_f_lockInBuffer = false;
    }

    // end of file reached? - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (m_f_eof) {       // m_f_eof and m_f_ID3v1TagFound will be set in playAudioData()
        if (m_f_loop) {  // file loop
            m_sumBytesDecoded = m_haveNewFilePos = m_audioDataStart;
            audiofile.seek(m_audioDataStart);
            InBuff.resetBuffer();
            AUDIO_INFO("file loop");
            m_f_eof = false;
            return;
        }
exit:
        char* afn = NULL;
        if(audiofile) afn = strdup(audiofile.name()); // store temporary the name
        stopSong();

        if(m_codec == CODEC_MP3) MP3Decoder_FreeBuffers();

        m_audioCurrentTime = 0;
        m_audioFileDuration = 0;
        m_resumeFilePos = -1;
        m_haveNewFilePos = 0;
        m_codec = CODEC_NONE;

        if(afn) {
            if (audio_eof_mp3) audio_eof_mp3(afn);
            AUDIO_INFO("End of file \"%s\"", afn);
            free(afn);
            afn = NULL;
        }

        return;
}
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::playAudioData() {
    if (!m_f_stream) return;  // guard

    static bool f_isFile = false;
    bool lastFrame = false;
    uint32_t bytesToDecode = 0;

    if (m_f_firstPlayCall) {
        m_f_firstPlayCall = false;
        f_isFile = true;
    }

    if (m_validSamples) {
        playChunk();
        return;
    }  // play samples first
    if (m_f_eof) return;

    if (m_f_lockInBuffer) return;
    m_f_audioTaskIsDecoding = true;
    uint8_t next = 0;
    int bytesDecoded = 0;
    if (f_isFile) {
        bytesToDecode = m_audioDataSize - m_sumBytesDecoded;
        if (bytesToDecode < InBuff.getMaxBlockSize()) {
            lastFrame = true;
        }
        if (m_sumBytesDecoded >= m_audioDataSize && m_sumBytesDecoded != 0) {
            m_f_eof = true;
            goto exit;
        }
    }

    if (!lastFrame) {
        if (InBuff.bufferFilled() < InBuff.getMaxBlockSize()) goto exit;
    }

    bytesDecoded = sendBytes(InBuff.getReadPtr(), InBuff.getMaxBlockSize());
    if (!m_f_running) return;

    if (bytesDecoded < 0) {  // no syncword found or decode error, try next chunk
        next = 200;
        if (InBuff.bufferFilled() < next) next = InBuff.bufferFilled();
        InBuff.bytesWasRead(next);  // try next chunk
        m_bytesNotDecoded += next;
        m_sumBytesDecoded += next;
    } else {
        if (bytesDecoded > 0) {
            InBuff.bytesWasRead(bytesDecoded);
            m_sumBytesDecoded += bytesDecoded;
            if (f_isFile && m_codec == CODEC_MP3) {
            }
            goto exit;
        }
        if (bytesDecoded == 0) goto exit;  // syncword at pos0
    }
exit:
    m_f_audioTaskIsDecoding = false;
    return;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::initializeDecoder(uint8_t codec) {
    uint32_t gfH = 0;
    uint32_t hWM = 0;
    switch(codec) {
        case CODEC_MP3:
            if (!MP3Decoder_IsInit()) {
                if (!MP3Decoder_AllocateBuffers()) {
                    AUDIO_INFO("The MP3Decoder could not be initialized");
                    goto exit;
                }
                gfH = ESP.getFreeHeap();
                hWM = uxTaskGetStackHighWaterMark(NULL);
                AUDIO_INFO("MP3Decoder has been initialized, free Heap: %lu bytes , free stack %lu DWORDs", (long unsigned int)gfH, (long unsigned int)hWM);
                InBuff.changeMaxBlockSize(m_frameSizeMP3);
            }
            break;
        case CODEC_WAV:
            InBuff.changeMaxBlockSize(m_frameSizeWav);
            break;
        default: goto exit; break;
    }
    return true;

exit:
    stopSong();
    return false;
}
void Audio::showCodecParams() {
    // print Codec Parameter (mp3, aac) in audio_info()

    AUDIO_INFO("Channels: %u", getChannels());
    AUDIO_INFO("SampleRate: %lu", (long unsigned int)getSampleRate());
    AUDIO_INFO("BitsPerSample: %u", getBitsPerSample());
    if(getBitRate()) { AUDIO_INFO("BitRate: %lu", (long unsigned int)getBitRate()); }
    else { AUDIO_INFO("BitRate: N/A"); }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int Audio::findNextSync(uint8_t* data, size_t len) {
    // Mp3 and aac audio data are divided into frames. At the beginning of each frame there is a sync word.
    // The sync word is 0xFFF. This is followed by information about the structure of the frame.
    // Wav files have no frames
    // Return: 0 the synchronous word was found at position 0
    //         > 0 is the offset to the next sync word
    //         -1 the sync word was not found within the block with the length len

    int         nextSync = 0;
    static uint32_t swnf = 0;
    if(m_codec == CODEC_WAV) {
        m_f_playing = true;
        nextSync = 0;
    }
    if(m_codec == CODEC_MP3) {
        nextSync = MP3FindSyncWord(data, len);
        if(nextSync == -1) return len; // syncword not found, search next block
    }
    if(nextSync == -1) {
        if(audio_info && swnf == 0) audio_info("syncword not found");
        else {
            swnf++; // syncword not found counter, can be multimediadata
        }
    }
    if(nextSync == 0) {
        if(audio_info && swnf > 0) {
            sprintf(m_chbuf, "syncword not found %lu times", (long unsigned int)swnf);
            audio_info(m_chbuf);
            swnf = 0;
        }
        else {
            if(audio_info) audio_info("syncword found at pos 0");
            m_f_decode_ready = true;
        }
    }
    if(nextSync > 0) { AUDIO_INFO("syncword found at pos %i", nextSync); }
    return nextSync;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::setDecoderItems() {
    if (m_codec == CODEC_MP3) {
        setChannels(MP3GetChannels());
        setSampleRate(MP3GetSampRate());
        setBitsPerSample(MP3GetBitsPerSample());
        setBitrate(MP3GetBitrate());
        AUDIO_INFO("MPEG-%s, Layer %s",(MP3GetVersion()==0) ? "2.5" : (MP3GetVersion()==2) ? "2" : "1", (MP3GetLayer()==1) ? "III" : (MP3GetLayer()==2) ? "II" : "I");
    }
    if (getBitsPerSample() != 8 && getBitsPerSample() != 16) {
        AUDIO_INFO("Bits per sample must be 8 or 16, found %i", getBitsPerSample());
        stopSong();
    }
    if (getChannels() != 1 && getChannels() != 2) {
        AUDIO_INFO("Num of channels must be 1 or 2, found %i", getChannels());
        stopSong();
    }
    reconfigI2S();
    showCodecParams();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int Audio::sendBytes(uint8_t* data, size_t len) {
    if(!m_f_running) return 0; // guard
    int32_t     bytesLeft;
    static bool f_setDecodeParamsOnce = true;
    int         nextSync = 0;
    if(!m_f_playing) {
        f_setDecodeParamsOnce = true;
        nextSync = findNextSync(data, len);
        if(nextSync == -1) return len;
        if(nextSync == 0) { m_f_playing = true; }
        return nextSync;
    }
    // m_f_playing is true at this pos
    bytesLeft = len;
    m_decodeError = 0;
    int bytesDecoded = 0;

    if(!m_f_decode_ready) return 0; // find sync first

    switch(m_codec) {
        case CODEC_WAV:  m_decodeError = 0; bytesLeft = 0; break;
        case CODEC_MP3:  m_decodeError = MP3Decode(data, &bytesLeft, m_outBuff, 0); break;
        default: {
            log_e("no valid codec found codec = %d", m_codec);
            stopSong();
        }
    }

    // m_decodeError - possible values are:
    //                   0: okay, no error
    //                 100: the decoder needs more data
    //                 < 0: there has been an error

    if(m_decodeError < 0) { // Error, skip the frame...

        printDecodeError(m_decodeError);
        m_f_playing = false; // seek for new syncword
        return 1; // skip one byte and seek for the next sync word
    }
    bytesDecoded = len - bytesLeft;

    if(bytesDecoded == 0 && m_decodeError == 0) { // unlikely framesize
        if(audio_info) audio_info("framesize is 0, start decoding again");
        m_f_playing = false; // seek for new syncword
        // we're here because there was a wrong sync word so skip one byte and seek for the next
        return 1;
    }
    // status: bytesDecoded > 0 and m_decodeError >= 0
    char* st = NULL;
    std::vector<uint32_t> vec;
    switch(m_codec) {
        case CODEC_WAV:     if(getBitsPerSample() == 16){
                                memmove(m_outBuff, data, len); // copy len data in outbuff and set validsamples and bytesdecoded=len
                                m_validSamples = len / (2 * getChannels());
                            }
                            else{
                                for(int i = 0; i < len; i++) {
                                    int16_t sample1 = (data[i] & 0x00FF)      - 128;
                                    int16_t sample2 = (data[i] & 0xFF00 >> 8) - 128;
                                    m_outBuff[i * 2 + 0] = sample1 << 8;
                                    m_outBuff[i * 2 + 1] = sample2 << 8;
                                }
                                m_validSamples = len;
                            }
                            break;
        case CODEC_MP3:     m_validSamples = MP3GetOutputSamps() / getChannels();
                            break;
    }
    if(f_setDecodeParamsOnce && m_validSamples) {
        f_setDecodeParamsOnce = false;
        setDecoderItems();
        m_PlayingStartTime = millis();
    }

    uint16_t bytesDecoderOut = m_validSamples;
    if(m_channels == 2) bytesDecoderOut /= 2;
    if(m_bitsPerSample == 16) bytesDecoderOut *= 2;
    computeAudioTime(bytesDecoded, bytesDecoderOut);

    m_curSample = 0;
    playChunk();
    return bytesDecoded;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::computeAudioTime(uint16_t bytesDecoderIn, uint16_t bytesDecoderOut) {
    static uint64_t sumBytesIn         = 0;
    static uint64_t sumBytesOut        = 0;
    static uint32_t sumBitRate         = 0;
    static uint32_t counter            = 0;
    static uint32_t timeStamp          = 0;
    static uint32_t deltaBytesIn       = 0;
    static uint32_t nominalBitRate     = 0;

    if(m_f_firstCurTimeCall) { // first call
        m_f_firstCurTimeCall = false;
        sumBytesIn = 0;
        sumBytesOut = 0;
        sumBitRate  = 0;
        counter = 0;
        timeStamp = millis();
        deltaBytesIn = 0;
        nominalBitRate = 0;

        if(m_codec == CODEC_WAV){
            nominalBitRate = getBitRate();
            m_avr_bitrate = nominalBitRate;
            m_audioFileDuration = m_audioDataSize  / (getSampleRate() * getChannels());
            if(getBitsPerSample() == 16) m_audioFileDuration /= 2;
        }
    }

    sumBytesIn   += bytesDecoderIn;
    deltaBytesIn += bytesDecoderIn;
    sumBytesOut  += bytesDecoderOut;

    if(timeStamp + 500 < millis()){
        uint32_t t       = millis();      // time tracking
        uint32_t delta_t = t - timeStamp; //    ---"---
        timeStamp = t;                    //    ---"---

        uint32_t bitRate = ((deltaBytesIn * 8000) / delta_t);  // we know the time and bytesIn to compute the bitrate

        sumBitRate += bitRate;
        counter ++;
        if(nominalBitRate){
            m_audioCurrentTime = round(((float)sumBytesIn * 8) / m_avr_bitrate);
        }
        else{
            m_avr_bitrate = sumBitRate / counter;
            m_audioCurrentTime = (sumBytesIn * 8) / m_avr_bitrate;
            m_audioFileDuration = round(((float)m_audioDataSize * 8 / m_avr_bitrate));
        }
        deltaBytesIn = 0;
    }

    if(m_haveNewFilePos && m_avr_bitrate){
        uint32_t posWhithinAudioBlock =  m_haveNewFilePos - m_audioDataStart;
        uint32_t newTime = posWhithinAudioBlock / (m_avr_bitrate / 8);
        m_audioCurrentTime = newTime;
        sumBytesIn = posWhithinAudioBlock;
        m_haveNewFilePos = 0;
    }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void Audio::printProcessLog(int r, const char* s){
    const char* e;
    const char* f = "";
    uint8_t logLevel;  // 1 Error, 2 Warn, 3 Info,
    switch(r) {
        case AUDIOLOG_PATH_IS_NULL: e = "The path ore file name is empty"; logLevel = 1; break;
        case AUDIOLOG_OUT_OF_MEMORY: e = "Out of memory"; logLevel = 1; break;
        case AUDIOLOG_FILE_NOT_FOUND: e = "File doesn't exist: "; logLevel = 1; f = s; break;
        case AUDIOLOG_FILE_READ_ERR: e = "Failed to open file for reading"; logLevel = 1; break;

        default: e = "UNKNOWN EVENT"; logLevel = 3; break;
    }
    if(audio_log){
        audio_log(logLevel, e, f);
    }
    else {
        if     (logLevel == 1) {AUDIO_INFO("ERROR: %s%s", e, f);}
        else if(logLevel == 2) {AUDIO_INFO("WARNING: %s%s", e, f);}
        else                   {AUDIO_INFO("INFO: %s%s", e, f);}
    }
 }
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::printDecodeError(int r) {
    const char* e;

    if (m_codec == CODEC_MP3) {
        switch(r) {
            case ERR_MP3_NONE: e = "NONE"; break;
            case ERR_MP3_INDATA_UNDERFLOW: e = "INDATA_UNDERFLOW"; break;
            case ERR_MP3_MAINDATA_UNDERFLOW: e = "MAINDATA_UNDERFLOW"; break;
            case ERR_MP3_FREE_BITRATE_SYNC: e = "FREE_BITRATE_SYNC"; break;
            case ERR_MP3_OUT_OF_MEMORY: e = "OUT_OF_MEMORY"; break;
            case ERR_MP3_NULL_POINTER: e = "NULL_POINTER"; break;
            case ERR_MP3_INVALID_FRAMEHEADER: e = "INVALID_FRAMEHEADER"; break;
            case ERR_MP3_INVALID_SIDEINFO: e = "INVALID_SIDEINFO"; break;
            case ERR_MP3_INVALID_SCALEFACT: e = "INVALID_SCALEFACT"; break;
            case ERR_MP3_INVALID_HUFFCODES: e = "INVALID_HUFFCODES"; break;
            case ERR_MP3_INVALID_DEQUANTIZE: e = "INVALID_DEQUANTIZE"; break;
            case ERR_MP3_INVALID_IMDCT: e = "INVALID_IMDCT"; break;
            case ERR_MP3_INVALID_SUBBAND: e = "INVALID_SUBBAND"; break;
            default: e = "ERR_UNKNOWN";
        }
        AUDIO_INFO("MP3 decode error %d : %s", r, e);
    }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::setPinout(uint8_t BCLK, uint8_t LRC, uint8_t DOUT, int8_t MCLK) {

    m_f_psramFound = psramInit();

    if(m_f_psramFound){ // shift mem in psram
        m_chbufSize = 4096;
        m_ibuffSize = 4096;
        x_ps_free(m_chbuf);
        x_ps_free(m_ibuff);
        x_ps_free(m_outBuff);
        m_outBuff  = (int16_t*)x_ps_malloc(m_outbuffSize * sizeof(int16_t));
        m_chbuf    = (char*)   x_ps_malloc(m_chbufSize);
        m_ibuff    = (char*)   x_ps_malloc(m_ibuffSize);
        if(!m_chbuf || !m_outBuff || !m_ibuff) log_e("oom");
    }
    esp_err_t result = ESP_OK;

    if(m_f_internalDAC) {
#if(ESP_IDF_VERSION_MAJOR != 5)
        i2s_set_pin((i2s_port_t)m_i2s_num,  NULL);
#endif
        return true;
    }

#if(ESP_ARDUINO_VERSION_MAJOR < 2)
    log_e("Arduino Version too old!");
#endif
#if(ESP_ARDUINO_VERSION_MAJOR == 2 && ESP_ARDUINO_VERSION_PATCH < 8)
    log_e("Arduino Version must be 2.0.8 or higher!");
#endif

#if(ESP_IDF_VERSION_MAJOR == 5)
    i2s_std_gpio_config_t gpio_cfg = {};
    gpio_cfg.bclk = (gpio_num_t)BCLK;
    gpio_cfg.din = (gpio_num_t)I2S_GPIO_UNUSED;
    gpio_cfg.dout = (gpio_num_t)DOUT;
    gpio_cfg.mclk = (gpio_num_t)MCLK;
    gpio_cfg.ws = (gpio_num_t)LRC;
    I2Sstop(m_i2s_num);
    result = i2s_channel_reconfig_std_gpio(m_i2s_tx_handle, &gpio_cfg);
    I2Sstart(m_i2s_num);
#else
    m_pin_config.bck_io_num = BCLK;
    m_pin_config.ws_io_num = LRC; //  wclk = lrc
    m_pin_config.data_out_num = DOUT;
    m_pin_config.data_in_num = I2S_GPIO_UNUSED;
    m_pin_config.mck_io_num = MCLK;
    result = i2s_set_pin((i2s_port_t)m_i2s_num, &m_pin_config);
#endif
    return (result == ESP_OK);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getFileSize() { // returns the size of webfile or local file
    return audiofile.size();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getFilePos() {
    if (!audiofile)
        return 0;
    return audiofile.position();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getAudioDataStartPos() {
    if (!audiofile)
        return 0;
    return m_audioDataStart;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getAudioFileDuration() {
    if(!m_avr_bitrate)                                      return 0;
    if(m_dataMode == AUDIO_LOCALFILE) {if(!m_audioDataSize) return 0;}
    return m_audioFileDuration;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getAudioCurrentTime() { // return current time in seconds
    return round(m_audioCurrentTime);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::setAudioPlayPosition(uint16_t sec) {
    // Jump to an absolute position in time within an audio file
    // e.g. setAudioPlayPosition(300) sets the pointer at pos 5 min
    if(sec > getAudioFileDuration()) sec = getAudioFileDuration();
    uint32_t filepos = m_audioDataStart + (m_avr_bitrate * sec / 8);
    return setFilePos(filepos);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::setVolumeSteps(uint8_t steps) {
    m_vol_steps = steps;
    if(steps < 1) m_vol_steps = 64; /* avoid div-by-zero :-) */
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t Audio::maxVolume() { return m_vol_steps; };
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getTotalPlayingTime() {
    // Is set to zero by a connectToXXX() and starts as soon as the first audio data is available,
    // the time counting is not interrupted by a 'pause / resume' and is not reset by a fileloop
    return millis() - m_PlayingStartTime;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::setTimeOffset(int sec) { // fast forward or rewind the current position in seconds
    if(m_dataMode != AUDIO_LOCALFILE) return false;
    if(m_dataMode == AUDIO_LOCALFILE && !audiofile) return false;
    if(!m_avr_bitrate) return false;
    uint32_t oneSec = m_avr_bitrate / 8;                 // bytes decoded in one sec
    int32_t  offset = oneSec * sec;                      // bytes to be wind/rewind
    uint32_t startAB = m_audioDataStart;                 // audioblock begin
    uint32_t endAB = m_audioDataStart + m_audioDataSize; // audioblock end

    int32_t pos = getFilePos() - inBufferFilled();
    pos += offset;
    if(pos < (int32_t)startAB) {pos = startAB;}
    if(pos >= (int32_t)endAB)  {pos = endAB;}
    setFilePos(pos);

    return true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::setFilePos(uint32_t pos) {
    if(m_dataMode == AUDIO_LOCALFILE && !audiofile) return false;
    memset(m_outBuff, 0, m_outbuffSize * sizeof(int16_t));
    m_validSamples = 0;
    m_haveNewFilePos = pos; // used in computeAudioCurrentTime()
    if(m_dataMode == AUDIO_LOCALFILE){
        m_resumeFilePos = pos;  // used in processLocalFile()
        return true;
    }
    return false;
}

bool Audio::audioFileSeek(const float speed) {
    // 0.5 is half speed
    // 1.0 is normal speed
    // 1.5 is one and half speed
//     if((speed > 1.5f) || (speed < 0.25f)) return false;

//     uint32_t srate = getSampleRate() * speed;
// #if ESP_IDF_VERSION_MAJOR == 5
//     I2Sstop(m_i2s_num);
//     m_i2s_std_cfg.clk_cfg.sample_rate_hz = srate;
//     i2s_channel_reconfig_std_clock(m_i2s_tx_handle, &m_i2s_std_cfg.clk_cfg);
//     I2Sstart(m_i2s_num);
// #else
//     i2s_set_sample_rates((i2s_port_t)m_i2s_num, srate);
// #endif
    return true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::setSampleRate(uint32_t sampRate) {
    if(!sampRate) sampRate = 44100; // fuse, if there is no value -> set default #209
    m_sampleRate = sampRate;
    return true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getSampleRate() { return m_sampleRate; }
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool Audio::setBitsPerSample(int bits) {
    if((bits != 16) && (bits != 8)) return false;
    m_bitsPerSample = bits;
    return true;
}
uint8_t Audio::getBitsPerSample() { return m_bitsPerSample; }

bool Audio::setChannels(int ch) {
    m_channels = ch;
    return true;
}
uint8_t Audio::getChannels() {
    if (m_channels == 0) {  // this should not happen! #209
        m_channels = 2;
    }
    return m_channels;
}

void Audio::reconfigI2S() {
#if ESP_IDF_VERSION_MAJOR == 5
    I2Sstop(0);

    if (getBitsPerSample() == 8 && getChannels() == 2)
        m_i2s_std_cfg.clk_cfg.sample_rate_hz = getSampleRate() * 2;
    else
        m_i2s_std_cfg.clk_cfg.sample_rate_hz = getSampleRate();

    if (!m_f_commFMT)
        m_i2s_std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    else
        m_i2s_std_cfg.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);

    m_i2s_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;

    i2s_channel_reconfig_std_clock(m_i2s_tx_handle, &m_i2s_std_cfg.clk_cfg);
    i2s_channel_reconfig_std_slot(m_i2s_tx_handle, &m_i2s_std_cfg.slot_cfg);

    I2Sstart(m_i2s_num);
#else
    m_i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    i2s_set_clk((i2s_port_t)m_i2s_num, m_sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
#endif
    memset(m_filterBuff, 0, sizeof(m_filterBuff));  // Clear FilterBuffer
    return;
}

bool Audio::setBitrate(int br) {
    m_bitRate = br;
    if (br) return true;
    return false;
}

uint32_t Audio::getBitRate(bool avg) {
    if (avg) return m_avr_bitrate;
    return m_bitRate;
}

void Audio::setI2SCommFMT_LSB(bool commFMT) {
    // false: I2S communication format is by default I2S_COMM_FORMAT_I2S_MSB, right->left (AC101, PCM5102A)
    // true:  changed to I2S_COMM_FORMAT_I2S_LSB for some DACs (PT8211)
    //        Japanese or called LSBJ (Least Significant Bit Justified) format

    m_f_commFMT = commFMT;

#if ESP_IDF_VERSION_MAJOR < 5
    if(commFMT) {
        AUDIO_INFO("commFMT = LSBJ (Least Significant Bit Justified)");
        m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_MSB);
    }
    else {
        AUDIO_INFO("commFMT = Philips");
        m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S);
    }
    i2s_driver_uninstall((i2s_port_t)m_i2s_num);
    i2s_driver_install((i2s_port_t)m_i2s_num, &m_i2s_config, 0, NULL);
#else
    i2s_channel_disable(m_i2s_tx_handle);
    if(commFMT) {
        AUDIO_INFO("commFMT = LSBJ (Least Significant Bit Justified)");
        m_i2s_std_cfg.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    }
    else {
        AUDIO_INFO("commFMT = Philips");
        m_i2s_std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    }
    i2s_channel_reconfig_std_slot(m_i2s_tx_handle, &m_i2s_std_cfg.slot_cfg);
    i2s_channel_enable(m_i2s_tx_handle);
#endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::computeVUlevel(int16_t sample[2]) {
    static uint8_t sampleArray[2][4][8] = {0};
    static uint8_t cnt0 = 0, cnt1 = 0, cnt2 = 0, cnt3 = 0, cnt4 = 0;
    static bool    f_vu = false;

    auto avg = [&](uint8_t* sampArr) { // lambda, inner function, compute the average of 8 samples
        uint16_t av = 0;
        for(int i = 0; i < 8; i++) { av += sampArr[i]; }
        return av >> 3;
    };

    auto largest = [&](uint8_t* sampArr) { // lambda, inner function, compute the largest of 8 samples
        uint16_t maxValue = 0;
        for(int i = 0; i < 8; i++) {
            if(maxValue < sampArr[i]) maxValue = sampArr[i];
        }
        return maxValue;
    };

    if(cnt0 == 64) {
        cnt0 = 0;
        cnt1++;
    }
    if(cnt1 == 8) {
        cnt1 = 0;
        cnt2++;
    }
    if(cnt2 == 8) {
        cnt2 = 0;
        cnt3++;
    }
    if(cnt3 == 8) {
        cnt3 = 0;
        cnt4++;
        f_vu = true;
    }
    if(cnt4 == 8) { cnt4 = 0; }

    if(!cnt0) { // store every 64th sample in the array[0]
        sampleArray[LEFTCHANNEL][0][cnt1] = abs(sample[LEFTCHANNEL] >> 7);
        sampleArray[RIGHTCHANNEL][0][cnt1] = abs(sample[RIGHTCHANNEL] >> 7);
    }
    if(!cnt1) { // store argest from 64 * 8 samples in the array[1]
        sampleArray[LEFTCHANNEL][1][cnt2] = largest(sampleArray[LEFTCHANNEL][0]);
        sampleArray[RIGHTCHANNEL][1][cnt2] = largest(sampleArray[RIGHTCHANNEL][0]);
    }
    if(!cnt2) { // store avg from 64 * 8 * 8 samples in the array[2]
        sampleArray[LEFTCHANNEL][2][cnt3] = largest(sampleArray[LEFTCHANNEL][1]);
        sampleArray[RIGHTCHANNEL][2][cnt3] = largest(sampleArray[RIGHTCHANNEL][1]);
    }
    if(!cnt3) { // store avg from 64 * 8 * 8 * 8 samples in the array[3]
        sampleArray[LEFTCHANNEL][3][cnt4] = avg(sampleArray[LEFTCHANNEL][2]);
        sampleArray[RIGHTCHANNEL][3][cnt4] = avg(sampleArray[RIGHTCHANNEL][2]);
    }
    if(f_vu) {
        f_vu = false;
        m_vuLeft = avg(sampleArray[LEFTCHANNEL][3]);
        m_vuRight = avg(sampleArray[RIGHTCHANNEL][3]);
    }
    cnt1++;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint16_t Audio::getVUlevel() {
    // avg 0 ... 127
    if(!m_f_running) return 0;
    return (m_vuLeft << 8) + m_vuRight;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::forceMono(bool m) { // #100 mono option
    m_f_forceMono = m;          // false stereo, true mono
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::setBalance(int8_t bal) { // bal -16...16
    if(bal < -16) bal = -16;
    if(bal > 16) bal = 16;
    m_balance = bal;

    computeLimit();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::setVolume(uint8_t vol, uint8_t curve) { // curve 0: default, curve 1: flat at the beginning

    uint16_t v = ESP_ARDUINO_VERSION_MAJOR * 100 + ESP_ARDUINO_VERSION_MINOR * 10 + ESP_ARDUINO_VERSION_PATCH;
    if(v < 207) AUDIO_INFO("Do not use this ancient Adruino version V%d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);

    if(vol > m_vol_steps) m_vol = m_vol_steps;
    else m_vol = vol;

    if(curve > 1) m_curve = 1;
    else m_curve = curve;

    computeLimit();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t Audio::getVolume() { return m_vol; }
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t Audio::getI2sPort() { return m_i2s_num; }
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::computeLimit() {    // is calculated when the volume or balance changes
    double l = 1, r = 1, v = 1; // assume 100%

    /* balance is left -16...+16 right */
    /* TODO: logarithmic scaling of balance, too? */
    if(m_balance > 0) { r -= (double)abs(m_balance) / 16; }
    else if(m_balance < 0) { l -= (double)abs(m_balance) / 16; }

    switch(m_curve) {
        case 0:
            v = (double)pow(m_vol, 2) / pow(m_vol_steps, 2); // square (default)
            break;
        case 1: // logarithmic
            double log1 = log(1);
            if(m_vol > 0) { v = m_vol * ((std::exp(log1 + (m_vol - 1) * (std::log(m_vol_steps) - log1) / (m_vol_steps - 1))) / m_vol_steps) / m_vol_steps; }
            else { v = 0; }
            break;
    }

    m_limit_left = l * v;
    m_limit_right = r * v;

    // log_i("m_limit_left %f,  m_limit_right %f ",m_limit_left, m_limit_right);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Audio::Gain(int16_t* sample) {
    /* important: these multiplications must all be signed ints, or the result will be invalid */
    sample[LEFTCHANNEL]  *= m_limit_left ;
    sample[RIGHTCHANNEL] *= m_limit_right;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::inBufferFilled() {
    // current audio input buffer fillsize in bytes
    return InBuff.bufferFilled();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t Audio::inBufferFree() {
    // current audio input buffer free space in bytes
    return InBuff.freeSpace();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

uint32_t Audio::inBufferSize() {
    // current audio input buffer size in bytes
    return InBuff.getBufsize();
}

int32_t Audio::mp3_correctResumeFilePos(uint32_t resumeFilePos) {

    // The SncronWord sequence 0xFF 0xF? can be part of valid audio data. Therefore, it cannot be ensured that the next 0xFFF is really the beginning
    // of a new MP3 frame. Therefore, the following byte is parsed. If the bitrate and sample rate match the one currently being played,
    // the beginning of a new MP3 frame is likely.

    const int16_t bitrateTab[3][3][15] PROGMEM = { {
        /* MPEG-1 */
        { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448 }, /* Layer 1 */
        { 0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384 }, /* Layer 2 */
        { 0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320 }, /* Layer 3 */
        }, {
        /* MPEG-2 */
        { 0, 32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256 }, /* Layer 1 */
        { 0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160 }, /* Layer 2 */
        { 0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160 }, /* Layer 3 */
        }, {
        /* MPEG-2.5 */
        { 0, 32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256 }, /* Layer 1 */
        { 0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160 }, /* Layer 2 */
        { 0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160 }, /* Layer 3 */
    }, };

    auto find_sync_word = [&](size_t pos) -> int {
        int steps = 0;
        audiofile.seek(pos);  // Set the file pointer to the given position
        while (audiofile.available()) {
            char c = audiofile.read();
            steps++;
            if (c == 0xFF) {  // First byte of sync word found
                char nextByte = audiofile.read();
                steps++;
                if ((nextByte & 0xF0) == 0xF0) {  // Check for the second part of sync word
                    return steps - 2;             // Sync word found, return the number of steps
                }
            }
        }
        return -1;  // Return -1 if sync word is not found
    };

    uint32_t pos = resumeFilePos;
    if (pos < m_audioDataStart) pos = m_audioDataStart;
    audiofile.seek(pos);
    uint8_t syncH, syncL, frame0;
    int steps;

    while (true) {
        steps = find_sync_word(pos);
        if (steps == -1) break;
        pos += steps;
        audiofile.seek(pos);
        syncH = audiofile.read();
        (void)syncH;
        syncL = audiofile.read();
        frame0 = audiofile.read();
        int32_t verIdx = (syncL >> 3) & 0x03;
        uint8_t mpegVers = (verIdx == 0 ? MPEG25 : ((verIdx & 0x01) ? MPEG1 : MPEG2));
        uint8_t brIdx = (frame0 >> 4) & 0x0f;
        uint8_t srIdx = (frame0 >> 2) & 0x03;
        uint8_t layer = 4 - ((syncL >> 1) & 0x03);
        if (srIdx == 3 || layer == 4 || brIdx == 15) {
            pos++;
            continue;
        }
        if (brIdx) {
            uint32_t bitrate = ((int32_t)bitrateTab[mpegVers][layer - 1][brIdx]) * 1000;
            uint32_t samplerate = samplerateTab[mpegVers][srIdx];
            //    log_e("%02X, %02X bitrate %i, samplerate %i", syncH, syncL, bitrate, samplerate);
            if (MP3GetBitrate() == bitrate && getSampleRate() == samplerate) break;
        }
        pos++;
    }
    return pos;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// separate task for decoding and outputting the data. 'playAudioData()' is started periodically and fetches the data from the InBuffer. This ensures
// that the I2S-DMA is always sufficiently filled, even if the Arduino 'loop' is stuck.
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Audio::setAudioTaskCore(uint8_t coreID){  // Recommendation:If the ARDUINO RUNNING CORE is 1, the audio task should be core 0 or vice versa
    if(coreID > 1) return;
    stopAudioTask();
    xSemaphoreTake(mutex_audioTask, 0.3 * configTICK_RATE_HZ);
    m_audioTaskCoreId = coreID;
    xSemaphoreGive(mutex_audioTask);
    startAudioTask();
}

void Audio::startAudioTask() {
    if (m_f_audioTaskIsRunning) {
        log_i("Task is already running.");
        return;
    }
    m_f_audioTaskIsRunning = true;

    m_audioTaskHandle = xTaskCreateStaticPinnedToCore(
        &Audio::taskWrapper,    /* Function to implement the task */
        "PeriodicTask",         /* Name of the task */
        AUDIO_STACK_SIZE,       /* Stack size in words */
        this,                   /* Task input parameter */
        2,                      /* Priority of the task */
        xAudioStack,            /* Task stack */
        &xAudioTaskBuffer,      /* Memory for the task's control block */
        m_audioTaskCoreId       /* Core where the task should run */
    );
}

void Audio::stopAudioTask()  {
    if (!m_f_audioTaskIsRunning) {
        log_i("audio task is not running.");
        return;
    }
    xSemaphoreTake(mutex_audioTask, 0.3 * configTICK_RATE_HZ);
    m_f_audioTaskIsRunning = false;
    if (m_audioTaskHandle != nullptr) {
        vTaskDelete(m_audioTaskHandle);
        m_audioTaskHandle = nullptr;
    }
    xSemaphoreGive(mutex_audioTask);
}

void Audio::taskWrapper(void *param) {
    Audio *runner = static_cast<Audio*>(param);
    runner->audioTask();
}

void Audio::audioTask() {
    while (m_f_audioTaskIsRunning) {
        vTaskDelay(1 / portTICK_PERIOD_MS);  // periodically every x ms
        performAudioTask();
    }
    vTaskDelete(nullptr);  // Delete this task
}

void Audio::performAudioTask() {
    if(!m_f_running) return;
    if(!m_f_stream) return;
    if(m_codec == CODEC_NONE) return; // wait for codec is set

    xSemaphoreTake(mutex_audioTask, 0.3 * configTICK_RATE_HZ);
    while (m_validSamples) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        playChunk();
    }  // I2S buffer full
    playAudioData();
    xSemaphoreGive(mutex_audioTask);
}

uint32_t Audio::getHighWatermark(){
    UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(m_audioTaskHandle);
    return highWaterMark; // dwords
}
