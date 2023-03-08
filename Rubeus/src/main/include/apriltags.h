/* This is the list of apriltags with defined offsets from 0 */

#define NUM_APRILTAGS_MAKERSPACE 6
#define NUM_APRILTAGS_OFFICIAL   8


const ApriltagPosition apriltags_makerspace[NUM_APRILTAGS_MAKERSPACE] = { // this is most certainly not a valid array; it's for testing here in the good ol' makerspace
  { 1, 0, 0                },
  { 5, 0, 1                },
  { 3, 0, -1.5             },
  { 6, 0, -3.125           },
  { 4, 3.1, -1, PI         },
  { 8, 3.1, -2.8, PI       }
};


const ApriltagPosition apriltags_official[NUM_APRILTAGS_OFFICIAL] = {
    {1, 0, 0},
    {2, 0, -1.65},
    {3, 0, -3.3},
    {4, -0.73, -5.51},
    {5, 14.53, -5.6, PI},
    {6, 13.8, -3.3, PI},
    {7, 13.8, -1.65, PI},
    {8, 13.8, 0, PI}
};

#define apriltags apriltags_official
#define NUM_APRILTAGS NUM_APRILTAGS_OFFICIAL