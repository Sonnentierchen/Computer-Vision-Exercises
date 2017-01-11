#include <iostream>
#include <queue>
using namespace std;
#include "ps.h"

double quality(int heightl, int widthl, unsigned char *imgl, int il, int jl,
               int heightr, int widthr, unsigned char *imgr, int ir, int jr,
               int wsize) {

    // filter out points that are close to borders
    if( il-wsize<0 ) return numeric_limits<double>::max();
    if( ir-wsize<0 ) return numeric_limits<double>::max();
    if( il+wsize>heightl-1 ) return numeric_limits<double>::max();
    if( ir+wsize>heightr-1 ) return numeric_limits<double>::max();
    if( jl-wsize<0 ) return numeric_limits<double>::max();
    if( jr-wsize<0 ) return numeric_limits<double>::max();
    if( jl+wsize>widthl-1 ) return numeric_limits<double>::max();
    if( jr+wsize>widthr-1 ) return numeric_limits<double>::max();

    double q=0.;
    for( int di=-wsize; di<=wsize; di++ ) {
        for( int dj=-wsize; dj<=wsize; dj++ ) {
            int indexl=((il+di)*widthl+jl+dj)*3;
            int indexr=((ir+di)*widthr+jr+dj)*3;
            double dr=(double)imgl[indexl]-(double)imgr[indexr];
            double dg=(double)imgl[indexl+1]-(double)imgr[indexr+1];
            double db=(double)imgl[indexl+2]-(double)imgr[indexr+2];
            q+=dr*dr+dg*dg+db*db;
        }
    }
    return q;
}

bool compareEntries(MARRIAGE_ENTRY &left, MARRIAGE_ENTRY &right) {
    return left.value < right.value;
}

void initMatches(int heightl, int widthl, unsigned char *imgl,
                 vector<KEYPOINT>* pointsl, int numberOfLeftPoints,
                 vector<vector<MARRIAGE_ENTRY>>* leftMatches,
                 int heightr, int widthr, unsigned char *imgr,
                 vector<KEYPOINT>* pointsr, int numberOfRightPoints,
                 vector<vector<MARRIAGE_ENTRY>>* rightMatches,
                 int wsize) {
    for(int i = 0; i < numberOfLeftPoints; i++){
        for(int j = 0; j < numberOfRightPoints; j++){
            double q = quality(heightl, widthl, imgl, (*pointsl)[i].y, (*pointsl)[i].x,
                               heightr, widthr, imgr, (*pointsr)[j].y, (*pointsr)[j].x,
                               wsize);
            if( q != numeric_limits<double>::max() ){
                MARRIAGE_ENTRY entry1;
                entry1.leftIndex = i;
                entry1.rightIndex = j;
                entry1.value = q;
                MARRIAGE_ENTRY entry2;
                entry2.leftIndex = i;
                entry2.rightIndex = j;
                entry2.value = q;
                // we have a possible match -> add the match
                (*leftMatches)[i].push_back(entry1);
                (*rightMatches)[j].push_back(entry2);
            }
        }
    }

    for (vector<MARRIAGE_ENTRY> _leftMatches : (*leftMatches)) {
        sort(_leftMatches.begin(), _leftMatches.end(), compareEntries);
    }

    for (vector<MARRIAGE_ENTRY> _rightMatches : (*rightMatches)) {
        sort(_rightMatches.begin(), _rightMatches.end(), compareEntries);
    }
}

vector<MATCH> matching(int heightl, int widthl, unsigned char *imgl,
                       int heightr, int widthr, unsigned char *imgr,
                       vector<KEYPOINT> pointsl, vector<KEYPOINT> pointsr, int wsize) {
    int numberOfPointsLeft = pointsl.size();
    int numberOfPointsRight = pointsr.size();

    // stores all matches of left points with right points
    vector<vector<MARRIAGE_ENTRY>> leftMatches(numberOfPointsLeft);
    // stores all matches of right points with left points
    vector<vector<MARRIAGE_ENTRY>> rightMatches(numberOfPointsRight);
    // stores whether a point on the left is already engaged
    vector<bool> leftEngaged(numberOfPointsLeft);
    // stores a proposal, i.e. a not finialized marriage of a right point
    vector<MARRIAGE_ENTRY> rightProposed(numberOfPointsRight);

    initMatches(heightl, widthl, imgl, &pointsl, numberOfPointsLeft, &leftMatches,
                heightr, widthr, imgr, &pointsr, numberOfPointsRight, &rightMatches,
                wsize);

    bool stillToMatch = true;
    while (stillToMatch) {
        stillToMatch = false;
        for (int i = 0; i < numberOfPointsLeft; i++) {
            // check whether the left side is free and has still matches left to check
            if (!leftEngaged[i] && leftMatches[i].size() > 0) {
                // select the topmost entry
                MARRIAGE_ENTRY leftEntry = leftMatches[i][0];
                if (rightProposed[leftEntry.rightIndex].leftIndex == -1) {
                    // right side is not proposed to anyone yet
                    // we add a new proposal
                    MARRIAGE_ENTRY newEntry;
                    newEntry.leftIndex = leftEntry.leftIndex;
                    newEntry.rightIndex = leftEntry.rightIndex;
                    newEntry.value = leftEntry.value;
                    rightProposed[leftEntry.rightIndex] = newEntry;
                    leftEngaged[i] = true;
                } else {
                    // the right side has already been proposed to a left point
                    // check whether we have to undo the proposal
                    MARRIAGE_ENTRY existingProposal = rightProposed[leftEntry.rightIndex];
                    if (existingProposal.value > leftEntry.value) {
                        // the entry that we currently check fits the right side better
                        leftEngaged[existingProposal.leftIndex] = false;
                        leftEngaged[i] = true;
                        MARRIAGE_ENTRY newEntry;
                        newEntry.leftIndex = i;
                        newEntry.rightIndex = leftEntry.rightIndex;
                        newEntry.value = leftEntry.value;
                        rightProposed[leftEntry.rightIndex] = newEntry;
                        stillToMatch = true;
                    }
                }
                leftMatches[i].erase(leftMatches[i].begin());
            }
        }
    }

    // fill
    vector<MATCH> result;
    for( int i = 0; i < numberOfPointsRight; i++ ) {
       MARRIAGE_ENTRY next = rightProposed[i];
       if (next.leftIndex != -1) {
           // there is a match
           KEYPOINT left = pointsl[next.leftIndex];
           KEYPOINT right = pointsr[next.rightIndex];
           MATCH newMatch;
           newMatch.xl = left.x;
           newMatch.yl = left.y;
           newMatch.xr = right.x;
           newMatch.yr = right.y;
           newMatch.value = next.value;
           result.push_back(newMatch);
       }
    }

    return result;
}
