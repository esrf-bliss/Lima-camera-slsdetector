#include "lima/Timestamp.h"
#include "lima/MiscUtils.h"
#include "lima/ThreadUtils.h"
#include "SlsDetectorCPUAffinity.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

DEB_GLOBAL(DebModTest);

class TestThread : public Thread {
	DEB_CLASS(DebModTest, "TestThread");
public:
	TestThread(int cpu, Cond& cond, volatile bool& active);

	int getCPU() const
	{ return m_cpu; }

	uint64_t getCount() const
	{ return m_count; }

protected:
	virtual void threadFunction();

private:
	AutoMutex lock()
	{ return m_cond.mutex(); }

	int m_cpu;
	Cond& m_cond;
	volatile bool& m_active;
	uint64_t m_count;
};


TestThread::TestThread(int cpu, Cond& cond, volatile bool& active)
	: m_cpu(cpu), m_cond(cond), m_active(active)
{
	DEB_CONSTRUCTOR();

	m_count = -1;

	start();

	{
		AutoMutex l = lock();
		while (m_count != 0)
			m_cond.wait();
	}

	CPUAffinity aff(1L << cpu);
	aff.applyToTask(getThreadID(), false, false);
}

void TestThread::threadFunction()
{
	DEB_MEMBER_FUNCT();

	{
		AutoMutex l = lock();
		m_count = 0;
		m_cond.broadcast();
		DEB_TRACE() << "Thread " << m_cpu << " started";
		while (!m_active)
			m_cond.wait();
	}
	DEB_TRACE() << "Thread " << m_cpu << " active";

	while (m_active) {
		for (int i = 0; i < 1000; ++i)
			;
		++m_count;
	}
}

int main(int argc, char *argv[])
{
	DEB_GLOBAL_FUNCT();

	bool debug = false;
	if (debug)
		DebParams::setTypeFlags(DebParams::AllFlags);

	int mask = 0;
	if (argc > 1) {
		istringstream is(argv[1]);
		is >> mask;
	}
	SystemCPUAffinityMgr mgr;

	mgr.setOtherCPUAffinity(CPUAffinity(0x001));

	NetDevGroupCPUAffinityList netdev_aff_list;
	NetDevGroupCPUAffinity netdev_aff;
	NetDevRxQueueCPUAffinity queue_aff;

	const char *netdev_group_0[] = {"eth0", "eth1", "eth2", "eth4",
					"eth6", "eth7", "eth8", "eth9"};
	netdev_aff.name_list = StringList(C_LIST_ITERS(netdev_group_0));
	queue_aff.processing = CPUAffinity(0x001);
	netdev_aff.queue_affinity[-1] = queue_aff;
	netdev_aff_list.push_back(netdev_aff);

	const char *netdev_group_1[] = {"eth3", "eth5"};
	netdev_aff.name_list = StringList(C_LIST_ITERS(netdev_group_1));
	queue_aff.processing = CPUAffinity(0x002);
	netdev_aff.queue_affinity[-1] = queue_aff;
	netdev_aff_list.push_back(netdev_aff);

	CPUAffinity this_aff = 0x001;
	DEB_TRACE() << "Setting " << DEB_VAR1(this_aff);
	this_aff.applyToTask(GetThreadID(), false, false);

	Cond cond;
	volatile bool active = false;

	int nb_cpus = CPUAffinity::getNbSystemCPUs();
	int test_cpus = mask ? mask : (1 << nb_cpus) - 1;
	DEB_ALWAYS() << DEB_VAR1(DebHex(test_cpus));
	typedef vector<AutoPtr<TestThread> > ThreadList;
	ThreadList thread_list;
	for (int i = 0; i < nb_cpus; ++i) {
		if (1 << i & test_cpus) {
			TestThread *t = new TestThread(i, cond, active);
			int cpu = t->getCPU();
			DEB_ALWAYS() << DEB_VAR1(cpu);
			thread_list.push_back(t);
		}
	}
	
	DEB_ALWAYS() << "Starting";
	active = true;
	cond.broadcast();

	Sleep(5);
	DEB_ALWAYS() << "Stopping";
	active = false;
	ThreadList::iterator it, end = thread_list.end();
	for (it = thread_list.begin(); it != end; ++it) {
		TestThread *t = *it;
		int cpu = t->getCPU();
		(*it)->join();
		float count = t->getCount();
		DEB_ALWAYS() << DEB_VAR2(cpu, count);
	}


	return 0;
}
