#pragma once

#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <fstream>

/**
 * History
 *
 * Class for queue past value and 
 * interpole them back in the past.
 */
class History
{
    public:

        /**
         * Initialization in timestamp duration
         */
        History(double window = 2.0);

        /**
         * Return the number of internal stored data
         */
        size_t size() const;

        /**
         * Return first and last recorded point
         */
        const std::pair<double, double>& front() const;
        const std::pair<double, double>& back() const;

        /**
         * Insert a new value in the container
         * with given timestamp and value
         */
        void pushValue(double timestamp, double value);

        /**
         * Return either the nearest value or the
         * linear interpolated value associated with
         * given timestamp
         */
        double interpolate(double timestamp) const;

        /**
         * Enable to logging mode.
         */
        void startLogging();

        /**
         * Stop logging and dump all recorded 
         * values into given output stream.
         * If binary is true, log file is written in
         * binary format
         */
        void stopLogging(std::ostream& os, bool binary = false);

        /**
         * Read data from given input stream 
         * until either the stream end or the first
         * commented "#" line.
         * If binary is true, log file is read in
         * binary format
         * Optional time shift is apply on read timestamp
         */
        void loadReplay(
            std::ifstream& is, 
            bool binary = false, 
            double timeShift = 0.0);

    private:

        /**
         * Mutex for concurent access
         */
        mutable std::mutex _mutex;

        /**
         * If true, the instance is in logging state
         * and does not erase any data
         */
        bool _isLogging;

        /**
         * When logging, save first time
         * to not write buffered data
         */
        double _startLoggingTime;

        /**
         * Rolling buffer size in timestamp
         */
        double _windowSize;

        /**
         * Values container indexed 
         * by their timestamp
         */
        std::deque<std::pair<double, double>> _values;
};

