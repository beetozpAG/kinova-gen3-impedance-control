#include <array>
#include <cstring>
#include <string>
#include <map>
// asio : Asynchronous Input/Output
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::udp;

struct Package
{
	char name[16];
	uint64_t timestamp;
	uint64_t remote_timestamp;
	uint32_t type;
	char data[100];
};

static constexpr unsigned int want_q = 0b1 << 0;
static constexpr unsigned int want_q_dot = 0b1 << 1;
static constexpr unsigned int want_pose = 0b1 << 2;
static constexpr unsigned int want_theta = 0b1 << 3;
static constexpr unsigned int want_theta_dot = 0b1 << 4;
static constexpr unsigned int want_tau = 0b1 << 5;

static constexpr unsigned int have_q = want_q << 16;
static constexpr unsigned int have_q_dot = want_q_dot << 16;
static constexpr unsigned int have_pose = want_pose << 16;
static constexpr unsigned int have_theta = want_theta << 16;
static constexpr unsigned int have_theta_dot = want_theta_dot << 16;
static constexpr unsigned int have_tau = want_tau << 16;

struct client_data {
	std::array<float, 7> q;
	std::array<float, 7> q_dot;
	std::array<float, 7> pose;
	std::array<float, 7> theta;
	std::array<float, 7> theta_dot;
	std::array<float, 7> tau;
	std::chrono::steady_clock::time_point t;
	uint64_t timestamp;
	uint64_t remote_timestamp;
	float delta_t;   
};

class udp_server
{
public:
	uint64_t timestamp;
	uint64_t remote_timestamp;
	std::array<float, 7> q;
	std::array<float, 7> q_dot;
	std::array<float, 7> pose;
	std::array<float, 7> theta;
	std::array<float, 7> theta_dot;
	std::array<float, 7> tau;

	std::array<float, 7> q_recv = {};
	std::array<float, 7> q_dot_recv = {};
	std::array<float, 7> pose_recv = {};
	std::array<float, 7> theta_recv = {};
	std::array<float, 7> theta_dot_recv = {};
	std::array<float, 7> tau_recv = {};
	std::map<std::string, client_data> clients;

private:
	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	std::string myName;
	std::array<char, sizeof(Package)> recv_buffer_;

public:
	udp_server(io_context &io_context, std::string name) : socket_(io_context, udp::endpoint(udp::v6(), 1313)), myName(name)
	{
		start_receive();
	}

private:
	void start_receive()
	{
		socket_.async_receive_from(buffer(recv_buffer_), remote_endpoint_, [this](std::error_code ec, std::size_t bytes)
								   { handle_receive(ec, bytes); });
	}
	void handle_receive(const std::error_code &error, std::size_t bytes_transfered)
	{
		if (!error && bytes_transfered >= sizeof(Package))
		{
			Package &recv_pkg = *reinterpret_cast<Package *>(recv_buffer_.data());
			// Receive Data (depending on 'Type'('have' section) from received package)
			int rcvData = 0;
			if (recv_pkg.type & have_q)
			{
				if (rcvData + sizeof(q_recv) < sizeof(Package::data))
					memcpy(&q_recv, recv_pkg.data + rcvData, sizeof(q_recv));
				rcvData += sizeof(q_recv);
			}
			if (recv_pkg.type & have_q_dot)
			{
				if (rcvData + sizeof(q_dot_recv) < sizeof(Package::data))
					memcpy(&q_dot_recv, recv_pkg.data  + rcvData, sizeof(q_dot_recv));
				rcvData += sizeof(q_dot_recv);
			}
			if (recv_pkg.type & have_pose)
			{
				if (rcvData + sizeof(pose_recv) < sizeof(Package::data))
					memcpy(&pose_recv, recv_pkg.data + rcvData, sizeof(pose_recv));
				rcvData += sizeof(pose_recv);
			}
			if (recv_pkg.type & have_theta)
			{
				if (rcvData + sizeof(theta_recv) < sizeof(Package::data))
					memcpy(&theta_recv, recv_pkg.data + rcvData, sizeof(theta_recv));
				rcvData += sizeof(theta_recv);
			}
			if (recv_pkg.type & have_theta_dot)
			{
				if (rcvData + sizeof(theta_dot_recv) < sizeof(Package::data))
					memcpy(&theta_dot_recv, recv_pkg.data + rcvData, sizeof(theta_dot_recv));
				rcvData += sizeof(theta_dot_recv);
			}
			if (recv_pkg.type & have_tau)
			{
				if (rcvData + sizeof(tau_recv) < sizeof(Package::data))
					memcpy(&tau_recv, recv_pkg.data + rcvData, sizeof(tau_recv));
				rcvData += sizeof(tau_recv);
			}
			auto prevClientTime = std::chrono::steady_clock::now();
			if(clients.count(std::string(recv_pkg.name))) {
				prevClientTime = clients[std::string(recv_pkg.name)].t;
			}
			client_data _data;
			_data.q = q_recv;
			_data.q_dot = q_dot_recv;
			_data.pose = pose_recv;
			_data.theta = theta_recv;
			_data.theta_dot = theta_dot_recv;
			_data.tau = tau_recv;
			_data.t = std::chrono::steady_clock::now();
			_data.delta_t = std::chrono::duration<float>(_data.t - prevClientTime).count();
			_data.timestamp = recv_pkg.timestamp;
			_data.remote_timestamp = recv_pkg.remote_timestamp;
			clients[std::string(recv_pkg.name)] = _data;

			// Send Data (which data depends on 'Type'('want' section) from last received package)
			Package *send_pkg = new Package;
			strncpy(send_pkg->name, myName.data(), sizeof(Package::name));
			send_pkg->timestamp = timestamp;
			send_pkg->remote_timestamp = remote_timestamp;
			send_pkg->type = recv_pkg.type;
			int currData = 0;
			if (recv_pkg.type & want_q)
			{
				// Copia de q al paquete
				if (currData + sizeof(q) < sizeof(Package::data))
					memcpy(send_pkg->data + currData, &q, sizeof(q));
				currData += sizeof(q);
			}
			if (recv_pkg.type & want_q_dot)
			{
				if (currData + sizeof(q_dot) < sizeof(Package::data))
					memcpy(send_pkg->data + currData, &q_dot, sizeof(q_dot));
				currData += sizeof(q_dot);
			}
			if (recv_pkg.type & want_pose)
			{
				if (currData + sizeof(pose) < sizeof(Package::data))
					memcpy(send_pkg->data + currData, &pose, sizeof(pose));
				currData += sizeof(pose);
			}
			if (recv_pkg.type & want_theta)
			{
				if (currData + sizeof(theta) < sizeof(Package::data))
					memcpy(send_pkg->data + currData, &theta, sizeof(theta));
				currData += sizeof(theta);
			}
			if (recv_pkg.type & want_theta_dot)
			{
				if (currData + sizeof(theta_dot) < sizeof(Package::data))
					memcpy(send_pkg->data + currData, &theta_dot, sizeof(theta_dot));
				currData += sizeof(theta_dot);
			}
			if (recv_pkg.type & want_tau)
			{
				if (currData + sizeof(tau) < sizeof(Package::data))
					memcpy(send_pkg->data + currData, &tau, sizeof(tau));
				currData += sizeof(tau);
			}
			socket_.async_send_to(buffer(send_pkg, sizeof(Package)), remote_endpoint_, [this, send_pkg](std::error_code, std::size_t)
								  { delete send_pkg; });
		}
		start_receive();
	}
};

class udp_client
{
public:
	uint64_t timestamp;
	uint64_t remote_timestamp;
	uint32_t type;
	std::array<float, 7> q;
	std::array<float, 7> q_dot;
	std::array<float, 7> pose;
	std::array<float, 7> theta;
	std::array<float, 7> theta_dot;
	std::array<float, 7> tau;

	std::array<float, 7> q_recv = {};
	std::array<float, 7> q_dot_recv = {};
	std::array<float, 7> pose_recv = {};
	std::array<float, 7> theta_recv = {};
	std::array<float, 7> theta_dot_recv = {};
	std::array<float, 7> tau_recv = {};
	uint64_t timestamp_recv;
	uint64_t remote_timestamp_recv;

private:
	int attempts = 0;
	udp::socket socket_;
	udp::endpoint server;
	high_resolution_timer timer;
	std::string myName;
	std::array<char, sizeof(Package)> recv_buffer_;

public:
	udp_client(io_context &io_context, udp::endpoint server, std::string name) : socket_{io_context, udp::endpoint(udp::v6(), 0)}, server{server}, timer{io_context}, myName{name}
	{
		do_send();
	}

private:
	void do_send()
	{
		Package *send_pkg = new Package;

		strncpy(send_pkg->name, myName.data(), sizeof(Package::name));
		send_pkg->timestamp = timestamp;
		send_pkg->remote_timestamp = remote_timestamp;
		send_pkg->type = type;
		int currData = 0;
		if (type & have_q)
		{
			// Copia de q al paquete
			if (currData + sizeof(q) < sizeof(Package::data))
				memcpy(send_pkg->data + currData, &q, sizeof(q));
			currData += sizeof(q);
		}
		if (type & have_q_dot)
		{
			if (currData + sizeof(q_dot) < sizeof(Package::data))
				memcpy(send_pkg->data + currData, &q_dot, sizeof(q_dot));
			currData += sizeof(q_dot);
		}
		if (type & have_pose)
		{
			if (currData + sizeof(pose) < sizeof(Package::data))
				memcpy(send_pkg->data + currData, &pose, sizeof(pose));
			currData += sizeof(pose);
		}
		if (type & have_theta)
		{
			if (currData + sizeof(theta) < sizeof(Package::data))
				memcpy(send_pkg->data + currData, &theta, sizeof(theta));
			currData += sizeof(theta);
		}
		if (type & have_theta_dot)
		{
			if (currData + sizeof(theta_dot) < sizeof(Package::data))
				memcpy(send_pkg->data + currData, &theta_dot, sizeof(theta_dot));
			currData += sizeof(theta_dot);
		}
		if (type & have_tau)
		{
			if (currData + sizeof(tau) < sizeof(Package::data))
				memcpy(send_pkg->data + currData, &tau, sizeof(tau));
			currData += sizeof(tau);
		}
		socket_.async_send_to(buffer(send_pkg, sizeof(Package)), server, [this, send_pkg](std::error_code ec, std::size_t)
							  { 
								if(!ec) {
								start_receive();
							  	}
								delete send_pkg; });
	}
	void handle_timeout(boost::system::error_code ec)
	{
		if (ec != error::operation_aborted)
		{

			socket_.cancel();
			if (++attempts >= 5)
			{
				std::cout << "Connection lost" << std::endl;
			}
			else
			{
				do_send();
			}
		}
	}

	void start_receive()
	{
		timer.expires_after(std::chrono::milliseconds(100));
		timer.async_wait([this](boost::system::error_code ec)
						 { handle_timeout(ec); });
		socket_.async_receive_from(buffer(recv_buffer_), server, [this](std::error_code ec, std::size_t bytes)
								   { handle_receive(ec, bytes); });
	}

	void handle_receive(const std::error_code &error, std::size_t bytes_transfered)
	{
		if (!error && bytes_transfered >= sizeof(Package))
		{
			timer.cancel();
			attempts = 0;
			Package &recv_pkg = *reinterpret_cast<Package *>(recv_buffer_.data());
			// Receive Data (depending on 'Type'('want' section) from answer package)
			int rcvData = 0;
			if (recv_pkg.type & want_q)
			{
				if (rcvData + sizeof(q_recv) < sizeof(Package::data))
					memcpy(&q_recv, recv_pkg.data + rcvData, sizeof(q_recv));
				rcvData += sizeof(q_recv);
			}
			if (recv_pkg.type & want_q_dot)
			{
				if (rcvData + sizeof(q_dot_recv) < sizeof(Package::data))
					memcpy(&q_dot_recv, recv_pkg.data + rcvData, sizeof(q_dot_recv));
				rcvData += sizeof(q_dot_recv);
			}
			if (recv_pkg.type & want_pose)
			{
				if (rcvData + sizeof(pose_recv) < sizeof(Package::data))
					memcpy(&pose_recv, recv_pkg.data + rcvData, sizeof(pose_recv));
				rcvData += sizeof(pose_recv);
			}
			if (recv_pkg.type & want_theta)
			{
				if (rcvData + sizeof(theta_recv) < sizeof(Package::data))
					memcpy(&theta_recv, recv_pkg.data + rcvData, sizeof(theta_recv));
				rcvData += sizeof(theta_recv);
			}
			if (recv_pkg.type & want_theta_dot)
			{
				if (rcvData + sizeof(theta_dot_recv) < sizeof(Package::data))
					memcpy(&theta_dot_recv, recv_pkg.data + rcvData, sizeof(theta_dot_recv));
				rcvData += sizeof(theta_dot_recv);
			}
			if (recv_pkg.type & want_tau)
			{
				if (rcvData + sizeof(tau_recv) < sizeof(Package::data))
					memcpy(&tau_recv, recv_pkg.data + rcvData, sizeof(tau_recv));
				rcvData += sizeof(tau_recv);
			}
			do_send();
			timestamp_recv = recv_pkg.timestamp;
			remote_timestamp_recv = recv_pkg.remote_timestamp;
		}
	}
};