# Localization
Ian Sodersjerna  
11/2/2022

## Goal
This package provides localization and navigation services for the In Home Delivery Bot
It can Receive Odometry information and ar marker information from the robot.
It then uses a particle filter to estimate the location of the robot on a known map.
It with then either run a waypoint mission or utilize A* pathfinding to navigate to the goal.

## Description