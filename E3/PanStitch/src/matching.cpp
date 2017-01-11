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

void createRanking(vector<KEYPOINT> pointsl, int numberOfPointsLeft,
                   vector<KEYPOINT> pointsr, int numberOfPointsRight,
                   vector<vector<MARRIAGE_PRIORITY_ENTRY>> *leftSidePriorities,
                   int heightl, int widthl, unsigned char *imgl,
                   int heightr, int widthr, unsigned char *imgr,
                   int wsize) {
    // create the list of right points for each left point by priority
    for ( int i = 0; i < numberOfPointsLeft; i++ ) {
        KEYPOINT leftPoint = pointsl[i];
        for ( int j = 0; j < numberOfPointsRight; j++ ) {
            KEYPOINT rightPoint = pointsr[j];
            double q = quality( heightl, widthl, imgl, leftPoint.x, leftPoint.y, heightr, widthr, imgr, rightPoint.x, rightPoint.y, wsize );
            MARRIAGE_PRIORITY_ENTRY rightPointEntry;
            rightPointEntry.value = q;
            rightPointEntry.index = j;
            // store the keypoint of the right together with the quality
            // the corresponding keypoint of the left is implicitly known by the index
            // we check where to insert the right keypoint according to its quality with the left keypoint
            // this way we obtain a prioritized list
            bool added = false;
            vector<MARRIAGE_PRIORITY_ENTRY> *priorities = &((*leftSidePriorities)[i]);
            for ( vector<MARRIAGE_PRIORITY_ENTRY>::iterator it = priorities->begin(); it != priorities->end(); it++ ) {
                MARRIAGE_PRIORITY_ENTRY current = *it;
                if ( rightPointEntry.value < current.value ) {
                    added = true;
                    priorities->insert( it, rightPointEntry );
                    break;
                }
            }
            if ( !added )
                priorities->push_back( rightPointEntry );
        }
    }
}

void marry(vector<KEYPOINT> pointsl, int numberOfPointsLeft, vector<vector<MARRIAGE_PRIORITY_ENTRY>> *leftSidePriorities,
           vector<KEYPOINT> pointsr, int numberOfPointsRight, vector<MARRIAGE_PRIORITY_ENTRY> *rightSideMatches) {

    vector<bool> rightSideMatched( numberOfPointsRight );
    fill( rightSideMatched.begin(), rightSideMatched.end(), false );
    vector<MARRIAGE_PRIORITY_ENTRY> _rightSideMatches = *rightSideMatches;

    // stores whether a keypoint of the left (i.e. the "men") has already been matched a partner
    vector<bool> leftSideMatched( numberOfPointsLeft );
    fill( leftSideMatched.begin(), leftSideMatched.end(), false );

    int stillToMarry = pointsl.size();

    while ( stillToMarry > 0 ) {

        int toMarryIndex;
        // find the first unengaged man
        for ( toMarryIndex = 0; toMarryIndex < numberOfPointsLeft; toMarryIndex++ ) {
            if ( !leftSideMatched[toMarryIndex] )
                break;
        }

        // loop through the prioritized possible right side matches for the current left side keypoint
        vector<vector<MARRIAGE_PRIORITY_ENTRY>> _leftSidePriorities = *leftSidePriorities;
        vector<MARRIAGE_PRIORITY_ENTRY> priorities = _leftSidePriorities[toMarryIndex];
        for ( vector<MARRIAGE_PRIORITY_ENTRY>::iterator it = priorities.begin();
              it != priorities.end() && !leftSideMatched[toMarryIndex]; it++ ) {
            // the prioritized left side keypoint of the current right side keypoint
            MARRIAGE_PRIORITY_ENTRY entry = *it;
            if ( !rightSideMatched[entry.index] ) {
                // the desired keypoint has not been matched yet, we are free to engage it
                rightSideMatched[entry.index] = true;
                leftSideMatched[toMarryIndex] = true;
                MARRIAGE_PRIORITY_ENTRY rightSideEntry;
                rightSideEntry.index = toMarryIndex;
                rightSideEntry.value = entry.value;
                _rightSideMatches[entry.index] = rightSideEntry;
                stillToMarry--;
            } else {
                // the desired left side keypoint is already engaged -> compare the values

                // the current engagement of the desired left side keypoint
                MARRIAGE_PRIORITY_ENTRY currentAssignment = _rightSideMatches[entry.index];
                if ( entry.value < currentAssignment.value ) {
                    // the left side keypoint prefers the right side keypoint that we check for over its current engagement
                    leftSideMatched[toMarryIndex] = true;
                    leftSideMatched[currentAssignment.index] = false;
                    _rightSideMatches[entry.index].index = toMarryIndex;
                    _rightSideMatches[entry.index].value = entry.value;
                }
            }
        }
    }
}

vector<MATCH> matching(int heightl, int widthl, unsigned char *imgl,
                       int heightr, int widthr, unsigned char *imgr,
                       vector<KEYPOINT> pointsl, vector<KEYPOINT> pointsr, int wsize) {
    int numberOfPointsLeft = pointsl.size();
    int numberOfPointsRight = pointsr.size();

    bool needToSwap = numberOfPointsLeft > numberOfPointsRight;

    // swap, because we marry the left side to the right side
    if (needToSwap) {
        swap(heightl, heightr);
        swap(widthl, widthr);
        swap(imgl, imgr);
        swap(pointsl, pointsr);
    }

    // stores the computed qualities for each keypoint of the left, i.e. contains a vector with the quality computed
    // with each keypoint of the right
    vector<vector<MARRIAGE_PRIORITY_ENTRY>> leftSidePriorities( numberOfPointsLeft );

    createRanking(pointsl, numberOfPointsLeft, pointsr, numberOfPointsRight,
                  &leftSidePriorities, heightl, widthl, imgl, heightr, widthr, imgr, wsize);

    // stores for each keypoint of the right (i.e. the "women") its matched partner from the left (i.e. the "men")
    // to get the partner of "woman" 4, use 4 as index
    vector<MARRIAGE_PRIORITY_ENTRY> rightSideMatches( numberOfPointsRight );

    marry(pointsl, numberOfPointsLeft, &leftSidePriorities,
          pointsr, numberOfPointsRight, &rightSideMatches);

    // fill
    vector<MATCH> result;
    for( int i = 0; i < rightSideMatches.size(); i++ ) {
        KEYPOINT right = pointsr[i];
        int left_index = rightSideMatches[i].index;
        KEYPOINT left = pointsl[left_index];
        MATCH toAdd;
        toAdd.value = rightSideMatches[i].value;
        KEYPOINT _right = needToSwap ? left : right;
        KEYPOINT _left = needToSwap ? right : left;
        toAdd.xl = _left.x;
        toAdd.yl = _left.y;
        toAdd.xr = _right.x;
        toAdd.yr = _right.y;
        result.push_back(toAdd);
    }

    return result;
}
