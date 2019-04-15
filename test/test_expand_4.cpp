
#include <SlsDetectorEiger.h>

#include <iomanip>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

DEB_GLOBAL(DebModTest);

void test_expand_4(bool raw)
{
	DEB_GLOBAL_FUNCT();

	cout << "raw=" << raw << endl;

	int nb_recvs = 2;
	int nb_proc_threads = 1;
	PixelDepth pixel_depth = PixelDepth4;
	Eiger::Geometry geom;
	geom.setNbRecvs(nb_recvs);
	for (int i = 0; i < nb_recvs; ++i)
		geom.getRecv(i)->setNbProcessingThreads(nb_proc_threads);
	geom.setRaw(raw);
	geom.setPixelDepth(pixel_depth);
	geom.setImageType(Bpp8);
	geom.prepareAcq();

	Eiger::Geometry::Recv *recv = geom.getRecv(0);
	Eiger::Geometry::Recv::Port *port = recv->getPort(0);
	FrameDim src_fdim = port->getSrcFrameDim();
	FrameDim dst_fdim = geom.getFrameDim(raw);
	FrameDim recv_fdim = geom.getRecvFrameDim(raw);
	int src_size = src_fdim.getMemSize();
	int dst_size = dst_fdim.getMemSize();
	int recv_size = recv_fdim.getMemSize();
	DEB_ALWAYS() << DEB_VAR6(src_fdim, src_size, dst_fdim, dst_size,
				 recv_fdim, recv_size);

	const int nb_recv_ports = recv->getNbPorts();
	typedef std::vector<MemBuffer> BufferList;
	BufferList src_buffer_list;
	src_buffer_list.resize(nb_recvs * nb_recv_ports);
	BufferList::iterator sit, send = src_buffer_list.end();
	int idx = 0;
	for (sit = src_buffer_list.begin(); sit != send; ++sit, ++idx) {
		sit->alloc(src_size);
		unsigned char *p = (unsigned char *) sit->getPtr();
		int dir = (idx % 2) ? -1 : 1;
		int chip_line = 0;
		for (int i = 0, v = 0; i < src_size; ++i, ++p) {
			if (i % 128 == 0)
				v = chip_line++;
			*p = (v + 1) << 4 | v;
			v = (v + 2 * dir) & 0xf;
		}
		cout << "Port #" << idx << endl;
		cout << setfill('0') << setbase(16);
		p = (unsigned char *) sit->getPtr();
		for (int l = 0; l < 8; ++l) {
			cout << setw(4) << l * 64 << ": ";
			for (int c = 0; c < 64; ++c, ++p)
				cout << setw(2) << int(*p) << " ";
			cout << endl;
		}
	}
	MemBuffer dst_buffer;
	dst_buffer.alloc(dst_size);

	int thread_idx = 0;
	sit = src_buffer_list.begin();
	for (int i = 0; i < nb_recvs; ++i) {
		recv = geom.getRecv(i);
		Eiger::Geometry::Recv::FrameData data;
		for (int j = 0; j < nb_recv_ports; ++j, ++sit) {
			data.src[j] = (char *) sit->getPtr();
			data.valid.set(j);
		}
		data.dst = (char *) dst_buffer.getPtr();
		recv->expandPixelDepth4(data, thread_idx);
		cout << "Recv #" << i << endl;
		cout << setfill('0') << setbase(16);
		unsigned char *p = (unsigned char *) dst_buffer.getPtr();
		p += recv_size * i;
		for (int l = 0; l < 32; ++l) {
			cout << setw(4) << l * 64 << ": ";
			for (int c = 0; c < 64; ++c, ++p)
				cout << setw(2) << int(*p) << " ";
			cout << endl;
		}
	}

	
}

int main(int argc, char *argv[])
{
	DEB_GLOBAL_FUNCT();

	test_expand_4(true);
	test_expand_4(false);

	return 0;
}
