#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <chrono>
#include <string>
#include <ctime>
#include <vector>
#include "nms.hpp"

void mergeObject(std::vector<Object>& boxes)
{
    if (boxes.size() >= 2)
    {
        for (int i = 0; i < boxes.size() - 1; i++) {
            for (int j = i + 1; j < boxes.size(); j++) {
                if ((boxes[i].rec & boxes[j].rec).area() > boxes[i].rec.area() * 3 / 5) {
                    boxes[i].rec = (boxes[i].rec | boxes[j].rec);
                    boxes.erase(boxes.begin() + j);
                    j--;
                }
            }
        }
    }
}

void sortBox(std::vector<Object>& boxes)
{
    Object index;
    if (boxes.size() < 2) return;
    for(int i = 0; i < boxes.size() - 1; i++)
    {
        for (int j = i + 1; j < boxes.size(); j++)
        {
            if (boxes[i].rec.x > boxes[j].rec.x) 
            {
                index = boxes[i];
                boxes[i] = boxes[j];
                boxes[j] = index;
            }
        }
    }
}

void sortObject(std::vector<Object>& boxes)
{
    int sumY = 0, avgY = 0;
    std::vector<Object> sortBox1, sortBox2;
    for(int i = 0; i < boxes.size(); i++)
    {
        sumY += boxes[i].rec.y;
    }
    avgY = sumY / boxes.size();
    for(int i = 0; i < boxes.size(); i++)
    {
        if (boxes[i].rec.y < avgY) 
        {
            sortBox1.push_back(boxes[i]);
        }
        else 
        {
            sortBox2.push_back(boxes[i]);
        }
    }
    sortBox(sortBox1);
    sortBox(sortBox2);
    sortBox1.insert(sortBox1.end(), sortBox2.begin(), sortBox2.end());

    boxes.clear();
    for (int i = 0; i < sortBox1.size(); i++)
    {
        boxes.push_back(sortBox1[i]);
    } 
}
