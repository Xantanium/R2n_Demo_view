//
// Created by xantanium on 19/5/25.
//

#pragma once

struct Transfer {
    bool transferUp : 1;
    bool transferDown : 1;
    bool sendBallToDribble : 1;

    void transferPipeline();

private:
    void manualTransfer();
    void autoTransfer();
};

