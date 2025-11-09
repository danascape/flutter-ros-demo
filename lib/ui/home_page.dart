import 'package:flutter/material.dart';
import 'dart:async';
import '../core/ros_service.dart';
import '../utils/message_utils.dart';

class RosHomePage extends StatefulWidget {
  const RosHomePage({super.key, required this.title});

  final String title;

  @override
  State<RosHomePage> createState() => _RosHomePageState();
}

class _RosHomePageState extends State<RosHomePage> {
  final RosService _rosService = RosService();
  final List<String> _receivedMessages = [];
  final TextEditingController _messageController = TextEditingController();
  final TextEditingController _publishTopicController = TextEditingController();
  final TextEditingController _subscribeTopicController = TextEditingController();
  final ScrollController _messagesScrollController = ScrollController();
  late StreamSubscription<String> _messageSubscription;
  bool _isConnected = false;

  @override
  void initState() {
    super.initState();
    // Set default topics
    _publishTopicController.text = '/flutter_messages';
    _subscribeTopicController.text = '/ros_messages';
    _initializeRos();
    _setupMessageListener();
  }

  void _initializeRos() async {
    try {
      await _rosService.initialize(
        publishTopic: _publishTopicController.text.trim(),
        subscribeTopic: _subscribeTopicController.text.trim(),
      );
      if (mounted) {
        setState(() {
          _isConnected = true;
          _receivedMessages.add("ROS node initialized successfully");
          _receivedMessages.add("Publishing to: ${_rosService.publishTopic}");
          _receivedMessages.add("Listening on: ${_rosService.subscribeTopic}");
        });
        _scrollToBottom();
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _isConnected = false;
          _receivedMessages.add("Error initializing ROS: $e");
        });
        _scrollToBottom();
      }
    }
  }

  void _setupMessageListener() {
    _messageSubscription = _rosService.messageStream.listen((message) {
      if (mounted) {
        setState(() {
          _receivedMessages.add(MessageUtils.formatReceivedMessage(message));
        });
        _scrollToBottom();
      }
    });
  }

  void _scrollToBottom() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_messagesScrollController.hasClients) {
        _messagesScrollController.animateTo(
          _messagesScrollController.position.maxScrollExtent,
          duration: const Duration(milliseconds: 300),
          curve: Curves.easeOut,
        );
      }
    });
  }

  void _sendMessage() async {
    final messageText = _messageController.text.trim();

    try {
      await _rosService.sendMessage(messageText);
      setState(() {
        _receivedMessages.add(MessageUtils.formatSentMessage(messageText));
      });
      _messageController.clear();
      _scrollToBottom();
    } catch (e) {
      setState(() {
        _receivedMessages.add("Error: ${e.toString().replaceFirst('RosPublishException: ', '')}");
      });
      _scrollToBottom();
    }
  }

  void _updateTopics() async {
    final publishTopic = _publishTopicController.text.trim();
    final subscribeTopic = _subscribeTopicController.text.trim();

    if (publishTopic.isEmpty || subscribeTopic.isEmpty) {
      setState(() {
        _receivedMessages.add("Error: Topic names cannot be empty");
      });
      return;
    }

    try {
      setState(() {
        _isConnected = false;
        _receivedMessages.add("Updating ROS topics...");
      });
      _scrollToBottom();

      await _rosService.updateTopics(publishTopic, subscribeTopic);

      setState(() {
        _isConnected = true;
        _receivedMessages.add("Topics updated successfully");
        _receivedMessages.add("Publishing to: ${_rosService.publishTopic}");
        _receivedMessages.add("Listening on: ${_rosService.subscribeTopic}");
      });
      _scrollToBottom();
    } catch (e) {
      setState(() {
        _isConnected = false;
        _receivedMessages.add("Error updating topics: $e");
      });
      _scrollToBottom();
    }
  }

  @override
  void dispose() {
    _messageSubscription.cancel();
    _messageController.dispose();
    _publishTopicController.dispose();
    _subscribeTopicController.dispose();
    _messagesScrollController.dispose();
    _rosService.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          children: [
            _buildTopicConfigCard(context),
            const SizedBox(height: 16),
            _buildMessageInputCard(context),
            const SizedBox(height: 16),
            _buildMessagesCard(context),
          ],
        ),
      ),
    );
  }

  Widget _buildTopicConfigCard(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Text(
                  'ROS Topic Configuration',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
                const Spacer(),
                Icon(
                  _isConnected ? Icons.radio_button_checked : Icons.radio_button_unchecked,
                  color: _isConnected ? Colors.green : Colors.grey,
                ),
                const SizedBox(width: 8),
                Text(
                  _isConnected ? 'Connected' : 'Disconnected',
                  style: TextStyle(
                    color: _isConnected ? Colors.green : Colors.grey,
                    fontSize: 14,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: TextField(
                    controller: _publishTopicController,
                    decoration: const InputDecoration(
                      labelText: 'Publish Topic',
                      border: OutlineInputBorder(),
                      hintText: '/flutter_messages',
                      helperText: 'Topic to publish messages to',
                    ),
                  ),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: TextField(
                    controller: _subscribeTopicController,
                    decoration: const InputDecoration(
                      labelText: 'Subscribe Topic',
                      border: OutlineInputBorder(),
                      hintText: '/ros_messages',
                      helperText: 'Topic to listen for messages',
                    ),
                  ),
                ),
                const SizedBox(width: 16),
                ElevatedButton(
                  onPressed: _updateTopics,
                  child: const Text('Update Topics'),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildMessageInputCard(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Send Message to ROS',
              style: Theme.of(context).textTheme.titleMedium,
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                Expanded(
                  child: TextField(
                    controller: _messageController,
                    decoration: InputDecoration(
                      labelText: 'Message',
                      border: const OutlineInputBorder(),
                      hintText: _isConnected ? 'Enter message to publish to ${_rosService.publishTopic}' : 'Connect to ROS first',
                    ),
                    enabled: _isConnected,
                  ),
                ),
                const SizedBox(width: 8),
                ElevatedButton(
                  onPressed: _isConnected ? _sendMessage : null,
                  child: const Text('Send'),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildMessagesCard(BuildContext context) {
    return Expanded(
      child: Card(
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'ROS Messages (from ${_rosService.subscribeTopic})',
                style: Theme.of(context).textTheme.titleMedium,
              ),
              const SizedBox(height: 8),
              Expanded(
                child: ListView.builder(
                  controller: _messagesScrollController,
                  itemCount: _receivedMessages.length,
                  itemBuilder: (context, index) {
                    return Padding(
                      padding: const EdgeInsets.symmetric(vertical: 2.0),
                      child: Text(
                        _receivedMessages[index],
                        style: const TextStyle(fontFamily: 'monospace', fontSize: 16),
                      ),
                    );
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}