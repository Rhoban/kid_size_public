#pragma once

#include <map>
#include <json/json.h>

class KickStrategy
{
    public:
        KickStrategy(double accuracy = 1);

        // Action
        struct Action
        {
            std::string kick;
            double orientation;
            double tolerance;
            double score;

            inline bool operator<(const Action &other) const {
                return score < other.score;
            }
        };

        // Action to take for a given goal
        Action actionFor(double x, double y);

        // Score for a given box
        double scoreFor(double x, double y);

        // Set the action
        void setAction(double x, double y, Action action);

        // Save to Json
        Json::Value toJson();

        // Load from Json
        bool fromJson(std::string filename);

        // plot
        void gnuplot();

        // Write data as a csv file
        void writeCSV(const std::string & path);

    protected:
        // Accuracies
        double accuracy;
        
        // Actions to take
        std::map<int, std::map<int, Action>> actions;
};
