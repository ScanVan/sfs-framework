//BSD 3-Clause License
//
//Copyright (c) 2017, JiaWang Bian
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//* Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//
//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//* Neither the name of the copyright holder nor the names of its
//  contributors may be used to endorse or promote products derived from
//  this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "gms_matcher.hpp"

int gms_matcher::GetInlierMask(vector<bool> &vbInliers, bool WithScale, bool WithRotation) {

	int max_inlier = 0;

	if (!WithScale && !WithRotation)
	{
		SetScale(0);
		max_inlier = run(1);
		vbInliers = mvbInlierMask;
		return max_inlier;
	}

	if (WithRotation && WithScale)
	{
		for (int Scale = 0; Scale < 5; Scale++)
		{
			SetScale(Scale);
			for (int RotationType = 1; RotationType <= 8; RotationType++)
			{
				int num_inlier = run(RotationType);

				if (num_inlier > max_inlier)
				{
					vbInliers = mvbInlierMask;
					max_inlier = num_inlier;
				}
			}
		}
		return max_inlier;
	}

	if (WithRotation && !WithScale)
	{
		SetScale(0);
		for (int RotationType = 1; RotationType <= 8; RotationType++)
		{
			int num_inlier = run(RotationType);

			if (num_inlier > max_inlier)
			{
				vbInliers = mvbInlierMask;
				max_inlier = num_inlier;
			}
		}
		return max_inlier;
	}

	if (!WithRotation && WithScale)
	{
		for (int Scale = 0; Scale < 5; Scale++)
		{
			SetScale(Scale);

			int num_inlier = run(1);

			if (num_inlier > max_inlier)
			{
				vbInliers = mvbInlierMask;
				max_inlier = num_inlier;
			}

		}
		return max_inlier;
	}

	return max_inlier;
}

void gms_matcher::AssignMatchPairs(int GridType) {

	for (size_t i = 0; i < mNumberMatches; i++)
	{
		Point2f &lp = mvP1[mvMatches[i].first];
		Point2f &rp = mvP2[mvMatches[i].second];

		int lgidx = mvMatchPairs[i].first = GetGridIndexLeft(lp, GridType);
		int rgidx = -1;

		if (GridType == 1)
		{
			rgidx = mvMatchPairs[i].second = GetGridIndexRight(rp);
		}
		else
		{
			rgidx = mvMatchPairs[i].second;
		}

		if (lgidx < 0 || rgidx < 0)	continue;

		mMotionStatistics.at<int>(lgidx, rgidx)++;
		mNumberPointsInPerCellLeft[lgidx]++;
	}

}

void gms_matcher::VerifyCellPairs(int RotationType) {

	const int *CurrentRP = mRotationPatterns[RotationType - 1];

	for (int i = 0; i < mGridNumberLeft; i++)
	{
		if (sum(mMotionStatistics.row(i))[0] == 0)
		{
			mCellPairs[i] = -1;
			continue;
		}

		int max_number = 0;
		for (int j = 0; j < mGridNumberRight; j++)
		{
			int *value = mMotionStatistics.ptr<int>(i);
			if (value[j] > max_number)
			{
				mCellPairs[i] = j;
				max_number = value[j];
			}
		}

		int idx_grid_rt = mCellPairs[i];

		const int *NB9_lt = mGridNeighborLeft.ptr<int>(i);
		const int *NB9_rt = mGridNeighborRight.ptr<int>(idx_grid_rt);

		int score = 0;
		double thresh = 0;
		int numpair = 0;

		for (size_t j = 0; j < 9; j++)
		{
			int ll = NB9_lt[j];
			int rr = NB9_rt[CurrentRP[j] - 1];
			if (ll == -1 || rr == -1)	continue;

			score += mMotionStatistics.at<int>(ll, rr);
			thresh += mNumberPointsInPerCellLeft[ll];
			numpair++;
		}

		thresh = THRESH_FACTOR * sqrt(thresh / numpair);

		if (score < thresh)
			mCellPairs[i] = -2;
	}
}

int gms_matcher::run(int RotationType) {

	mvbInlierMask.assign(mNumberMatches, false);

	// Initialize Motion Statisctics
	mMotionStatistics = Mat::zeros(mGridNumberLeft, mGridNumberRight, CV_32SC1);
	mvMatchPairs.assign(mNumberMatches, pair<int, int>(0, 0));

	for (int GridType = 1; GridType <= 4; GridType++)
	{
		// initialize
		mMotionStatistics.setTo(0);
		mCellPairs.assign(mGridNumberLeft, -1);
		mNumberPointsInPerCellLeft.assign(mGridNumberLeft, 0);

		AssignMatchPairs(GridType);
		VerifyCellPairs(RotationType);

		// Mark inliers
		for (size_t i = 0; i < mNumberMatches; i++)
		{
			if (mvMatchPairs[i].first >= 0) {
				if (mCellPairs[mvMatchPairs[i].first] == mvMatchPairs[i].second)
				{
					mvbInlierMask[i] = true;
				}
			}
		}
	}
	int num_inlier = sum(mvbInlierMask)[0];
	return num_inlier;
}
