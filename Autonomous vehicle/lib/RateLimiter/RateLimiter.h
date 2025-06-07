#pragma once

class RateLimiter {
    public:
        RateLimiter();
        void loadConfig(const char* configFile, const char* rateLimiterName);
        float updateRateLimiter(float input, float loopTime);

    
    private:
        float rateUp;
        float rateDown;
        float previousOutput;
};

