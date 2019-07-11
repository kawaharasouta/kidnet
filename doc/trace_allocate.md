# e1000

 - e1000_probe
	- e1000_sw_init
	 adapter->num_tx_queues = 1;
	 adapter->num_rx_queues = 1;
		- e1000_alloc_queue
		 adapter->tx_ring = kmalloc(adapter->num_tx_queues, sizeof(struct e1000_tx_ring), GFP_KERNEL);
		 adapter->rx_ring = kmalloc(adapter->num_rx_queues, sizeof(struct e1000_rx_ring), GFP_KERNEL);

 - e1000_open
	- e1000_setup_all_tx_resources
	 - e1000_setup_tx_resources
		adapter->tx_ring->buffer_info = vzalloc(sizeof(struct e1000_tx_buffer) * adapter->tx_ring->count);	#ringがi個あったらiでループ．
		adapter->tx_ring->size = adapter->tx_ring->count * sizeof(struct e1000_tx_desc);	
		adapter->tx_ring->size = ALIGN(adapter->tx_ring->size, 4096);	
		adapter->tx_ring->desc = dma_alloc _coherent(&pdev->dev, adapter->tx_ring->size, &adapter->tx_ring->dma, GFP_KERNEL);

	- e1000_setup_all_rx_resources
	あとでみにくる．

	- e1000_configure
	 - e1000_set_rx_mode 
		mcarrayっていうのがallocateしてあるけど何に使うかよくわからない．MTAってのがなんなのかどうか．
		E1000_WRITE_REG_ARRAY(hw, MTA, i, mcarray[i]);
		kfree(mcarray);
	 - e1000_configure_tx
	  デスクリプタリングのレジスタを初期化してるだけのはずなんだ．

	 - e1000_configure_rx
	  デスクリプタリングのレジスタを初期化してる．
		ただ，txと違ってadapter->alloc_rx_bufっていう関数ハンドラを初期化していて，
		
		
