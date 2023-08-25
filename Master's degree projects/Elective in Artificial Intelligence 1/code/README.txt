NOTE: we did not deliver also a zip containing the dataset, since it was too heavy
and we have used it through the servers provided by professor Giagu.

- "DatasetGeneration" contains the data analysis and preprocessing and the generation
  of the datasets that are used to then train both the DnCNN and the GANs.
- "CGAN" contains the code related to our proposed approach, namely the CGAN.
  There are the definition of the discriminator, the training pipeline and the
  evaluation of the results.
- "DnCNN_baseline" contains the implementation of the approach proposed by Ciardiello,
  that has been used as baseline.
- "DnCNN_comp" contains the implementation of the DnCNN that has been trained and tested
  in "DnCNN_baseline".
- "DnCNN_skip" contains the implementation of the generator that has been used in "CGAN".
- "utils" contains functions that are useful to build the reconstructed images from
  k-space data and for plotting coils data.