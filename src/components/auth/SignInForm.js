import React, { useState } from 'react';

const SignInForm = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
  });

  const { email, password } = formData;

  const onChange = (e) =>
    setFormData({ ...formData, [e.target.name]: e.target.value });

  const onSubmit = async (e) => {
    e.preventDefault();
    console.log('Sign In SUCCESS');
    // TODO: Integrate with backend API
  };

  return (
    <form onSubmit={onSubmit}>
      <div>
        <input
          type='email'
          placeholder='Email Address'
          name='email'
          value={email}
          onChange={onChange}
          required
        />
      </div>
      <div>
        <input
          type='password'
          placeholder='Password'
          name='password'
          value={password}
          onChange={onChange}
          minLength='6'
        />
      </div>
      <input type='submit' value='Sign In' />
    </form>
  );
};

export default SignInForm;