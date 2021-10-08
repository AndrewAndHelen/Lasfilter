#include "LasFilter.h"

static void Usage(const char* pszErrorMsg = NULL)
{
	fprintf(stderr, "options:\n");
	fprintf(stderr, "[-help,-h]						[produce help message]\n");
	fprintf(stderr, "[-szLasIn]					[input the absolute path of .las file]\n");
	fprintf(stderr, "[-szLasOut]					[output the absolute path of .las file]\n");
	fprintf(stderr, "[-lfLdrPtInterv]			[point density]\n");

	if (pszErrorMsg != NULL)
		fprintf(stderr, "\nFAILURE: %s\n", pszErrorMsg);

	exit(1);
}

int main(int argc, char *argv[])
{
	std::string szLasIn, szLasOut;
	double lfLdrPtInterv;

	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "-h") == 0)
		{
			Usage();
		}
		else if (strcmp(argv[i], "-szLasIn") == 0)
		{
			i++; if (i >= argc) continue;
			szLasIn = argv[i];
		}
		else if (strcmp(argv[i], "-szLasOut") == 0)
		{
			i++; if (i >= argc) continue;
			szLasOut = argv[i];
		}
		else if (strcmp(argv[i], "-lfLdrPtInterv") == 0) {
			i++; if (i >= argc) continue;
			lfLdrPtInterv = atof(argv[i]);
		}
		else
		{
			Usage("Too many command options.");
		}
	}

	LasFilter filter;
	filter.run(szLasIn, szLasOut, lfLdrPtInterv,true);

	return 0;
}