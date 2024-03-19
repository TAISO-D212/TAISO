import { useState, useEffect } from 'react';
import { Loading } from '../../components/home/Loading';

export const HomePage = () => {
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (loading) {
      setTimeout(() => {
        setLoading(false);
      }, 2000);
    }
  }, []);

  if (loading) {
    return (
      <div className="w-full h-screen flex-col justify-center items-center">
        <Loading />
      </div>
    );
  }

  return <>test</>;
};
