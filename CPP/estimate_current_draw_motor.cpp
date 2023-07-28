#include <iostream>

double estimateCurrentDraw(double stallCurrent, double stallLoad, double targetLoad) {
    double estimatedCurrent = (stallCurrent / stallLoad) * targetLoad;
    return estimatedCurrent;
}

int main() {
    try {
        double stallCurrent, stallLoad, targetLoad;
        
        std::cout << "Enter the stall current of the motor: ";
        std::cin >> stallCurrent;
        
        std::cout << "Enter the stall load of the motor: ";
        std::cin >> stallLoad;
        
        std::cout << "Enter the target load for current estimation: ";
        std::cin >> targetLoad;

        double estimatedCurrent = estimateCurrentDraw(stallCurrent, stallLoad, targetLoad);
        std::cout << "Estimated current draw at " << targetLoad << " load: " << estimatedCurrent << " A" << std::endl;

        if(estimatedCurrent > stallCurrent){
            std::cout<<"\033[0;31m W: Can't use , motor will burn\n";
        }


    } catch (...) {
        std::cout << "Invalid input. Please enter valid numbers." << std::endl;
    }

    return 0;
}
