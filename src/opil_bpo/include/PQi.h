#include <vector>

using namespace std;

template <class keyType> 
class PQi 
{ 
private:

	int d, N;
	vector<int> pq, qp; 
	const vector<keyType> &a;

	void exch(int i, int j)
	{ 
		int t = pq[i]; 
		pq[i] = pq[j]; 
		pq[j] = t;
		qp[pq[i]] = i; 
		qp[pq[j]] = j; 
	}

	void fixUp(int k)
	{ 
		while (k > 1 && a[pq[(k+d-2)/d]] > a[pq[k]])
		{ 
			exch(k, (k+d-2)/d); k = (k+d-2)/d; 
		} 
	}

	void fixDown(int k, int N)
	{ 
		int j;
		while ((j = d*(k-1)+2) <= N)
		{ 
			for (int i = j+1; i < j+d && i <= N; i++)
				if (a[pq[j]] > a[pq[i]]) 
					j = i;

			if (!(a[pq[k]] > a[pq[j]])) 
				break;

			exch(k, j); 
			k = j;
		}
	}

public:

	PQi(int N, const vector<keyType> &a, int d = 3) : 
		a(a), 
		pq(N+1, 0), 
		qp(N+1, 0), 
		N(0), 
		d(d) { }

	int empty() const { return N == 0; }
	
	void insert(int v) 
	{ 
		pq[++N] = v; 
		qp[v] = N; 
		fixUp(N); 
	}

	int getmin()
	{ 
		exch(1, N); 
		fixDown(1, N-1); 
		return pq[N--]; 
	}

	void lower(int k)
	{ 
		fixUp(qp[k]);
	}
};