-- ChatKit Database Schema Migration
-- Based on OpenAI ChatKit Store interface specification
-- Supports ThreadMetadata and ThreadItem models with pagination

-- ============================================================================
-- THREADS TABLE
-- ============================================================================
-- Stores thread metadata (conversations)
CREATE TABLE IF NOT EXISTS chatkit_threads (
    id VARCHAR(255) PRIMARY KEY,
    user_id VARCHAR(255) NOT NULL,
    metadata JSONB DEFAULT '{}'::jsonb,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_chatkit_threads_user ON chatkit_threads(user_id);
CREATE INDEX IF NOT EXISTS idx_chatkit_threads_created ON chatkit_threads(created_at DESC);


-- ============================================================================
-- THREAD ITEMS TABLE
-- ============================================================================
-- Stores all thread items: messages, tool_calls, tasks, workflows, attachments
-- Based on ChatKit's ThreadItem discriminated union type
CREATE TABLE IF NOT EXISTS chatkit_thread_items (
    id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) NOT NULL REFERENCES chatkit_threads(id) ON DELETE CASCADE,
    item_type VARCHAR(50) NOT NULL CHECK (item_type IN (
        'message',
        'tool_call',
        'tool_result',
        'task',
        'workflow',
        'attachment'
    )),

    -- Message fields (when item_type = 'message')
    role VARCHAR(50) CHECK (role IN ('user', 'assistant', 'system') OR role IS NULL),
    content TEXT,

    -- Tool call fields (when item_type = 'tool_call')
    tool_name VARCHAR(255),
    tool_arguments JSONB,
    tool_call_id VARCHAR(255),

    -- Tool result fields (when item_type = 'tool_result')
    tool_result TEXT,
    tool_result_call_id VARCHAR(255),

    -- Task fields (when item_type = 'task')
    task_name VARCHAR(255),
    task_status VARCHAR(50) CHECK (task_status IN ('pending', 'running', 'completed', 'failed') OR task_status IS NULL),
    task_result JSONB,

    -- Workflow fields (when item_type = 'workflow')
    workflow_name VARCHAR(255),
    workflow_status VARCHAR(50) CHECK (workflow_status IN ('pending', 'running', 'completed', 'failed') OR workflow_status IS NULL),
    workflow_steps JSONB,

    -- Attachment fields (when item_type = 'attachment')
    attachment_url TEXT,
    attachment_mime_type VARCHAR(100),
    attachment_size_bytes INTEGER,

    -- Common metadata
    metadata JSONB DEFAULT '{}'::jsonb,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Sequence for ordering items within thread
    sequence_number SERIAL
);

CREATE INDEX IF NOT EXISTS idx_chatkit_items_thread ON chatkit_thread_items(thread_id);
CREATE INDEX IF NOT EXISTS idx_chatkit_items_type ON chatkit_thread_items(item_type);
CREATE INDEX IF NOT EXISTS idx_chatkit_items_created ON chatkit_thread_items(created_at);
CREATE INDEX IF NOT EXISTS idx_chatkit_items_sequence ON chatkit_thread_items(thread_id, sequence_number);


-- ============================================================================
-- PAGINATION CURSOR TABLE
-- ============================================================================
-- Stores pagination state for thread item queries (supports Page[ThreadItem])
CREATE TABLE IF NOT EXISTS chatkit_pagination_cursors (
    cursor_id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) NOT NULL REFERENCES chatkit_threads(id) ON DELETE CASCADE,
    last_item_id VARCHAR(255),
    page_size INTEGER DEFAULT 50,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP + INTERVAL '1 hour')
);

CREATE INDEX IF NOT EXISTS idx_chatkit_cursors_thread ON chatkit_pagination_cursors(thread_id);
CREATE INDEX IF NOT EXISTS idx_chatkit_cursors_expires ON chatkit_pagination_cursors(expires_at);


-- ============================================================================
-- HELPER FUNCTIONS
-- ============================================================================

-- Function to automatically update updated_at timestamp
CREATE OR REPLACE FUNCTION update_chatkit_thread_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Trigger to update thread timestamp when items are added
CREATE TRIGGER trigger_update_thread_timestamp
AFTER INSERT ON chatkit_thread_items
FOR EACH ROW
EXECUTE FUNCTION update_chatkit_thread_timestamp();


-- ============================================================================
-- CLEANUP FUNCTION
-- ============================================================================
-- Function to clean expired pagination cursors (call periodically)
CREATE OR REPLACE FUNCTION cleanup_expired_pagination_cursors()
RETURNS INTEGER AS $$
DECLARE
    deleted_count INTEGER;
BEGIN
    DELETE FROM chatkit_pagination_cursors
    WHERE expires_at < CURRENT_TIMESTAMP;

    GET DIAGNOSTICS deleted_count = ROW_COUNT;
    RETURN deleted_count;
END;
$$ LANGUAGE plpgsql;


-- ============================================================================
-- COMMENTS (Documentation)
-- ============================================================================
COMMENT ON TABLE chatkit_threads IS 'ChatKit conversation threads with metadata';
COMMENT ON TABLE chatkit_thread_items IS 'ChatKit thread items supporting messages, tool calls, tasks, workflows, and attachments';
COMMENT ON TABLE chatkit_pagination_cursors IS 'Pagination state for thread item queries';

COMMENT ON COLUMN chatkit_thread_items.item_type IS 'Discriminator for ThreadItem union type: message, tool_call, tool_result, task, workflow, attachment';
COMMENT ON COLUMN chatkit_thread_items.sequence_number IS 'Auto-incrementing sequence for ordering items within a thread';


-- ============================================================================
-- VALIDATION QUERIES
-- ============================================================================
-- Verify tables were created successfully
DO $$
BEGIN
    IF NOT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_name = 'chatkit_threads') THEN
        RAISE EXCEPTION 'Failed to create chatkit_threads table';
    END IF;

    IF NOT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_name = 'chatkit_thread_items') THEN
        RAISE EXCEPTION 'Failed to create chatkit_thread_items table';
    END IF;

    IF NOT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_name = 'chatkit_pagination_cursors') THEN
        RAISE EXCEPTION 'Failed to create chatkit_pagination_cursors table';
    END IF;

    RAISE NOTICE 'ChatKit schema migration completed successfully';
END $$;
