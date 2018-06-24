#include "CppUnitTest.h"
#include "../nvFlexDop/NvFlexCollisionData.h"


using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace NvFlexUnitTest
{
	TEST_CLASS(UnitTest1)
	{
	public:

		TEST_METHOD(NvFlexCollisionDataTests)
		{
			NvFlexLibrary *lib = NvFlexInit();
			Assert::IsNotNull(lib);

			NvFlexCollisionData dat(lib);
			Assert::AreEqual(dat.size(), 0);

			dat.addSphere("woof");
			Assert::AreEqual(dat.size(), 1);

		}

	};
}